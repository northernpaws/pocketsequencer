// TODO: integrate https://github.com/okhsunrog/lcd-async/tree/master to add async

pub mod fmc;

use core::fmt::Pointer;

use defmt::{info, trace};

use embassy_stm32::{
    Peri,
    dma::{self, AnyChannel, Burst, FifoThreshold},
    exti::ExtiInput,
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::DrawTarget};
use embedded_graphics::{prelude::*, primitives::Rectangle};
use embedded_graphics_framebuf::{FrameBuf, backends::DMACapableFrameBufferBackend};

use crate::hardware::display::fmc::Fmc;

// NOTE: We run the display in it's portrait resolution because that's the direction
//  the panel scans in, regardless of rotational settings on the controller. If we
//  adjust the memory configuration to be in landscape then you get nasty diagonal
//  tearing on the display because the scanlines and updated memory run diagonal to
//  each other.
//
// Instead, the display is ran in it's native orientation, but rotational transforms
//  are applied to the display's draw target so that drawing operations are pre-rotated
//  to align with the display's portrait orientation, even though the coordinates are
//  written for landscape orientation.
pub const WIDTH: usize = 240;
pub const HEIGHT: usize = 320;
pub const FRAMEBUFFER_SIZE: usize = WIDTH * HEIGHT;
pub type FramebufferArray = [Rgb565; FRAMEBUFFER_SIZE];
pub type Framebuffer<'a> = FrameBuf<Rgb565, &'a mut FramebufferArray>;

pub struct Display<'a, DELAY: embedded_hal_async::delay::DelayNs> {
    interface: Fmc<'a>,
    rst: embassy_stm32::gpio::Output<'a>,
    te: ExtiInput<'a>,
    delay: DELAY,
    /// Embedded-graphics compatible DrawTarget framebuffer for updating the display.
    ///
    /// A framebuffer is used instead of drawing to the display directly
    /// to allow full-frame updates that are DMA transfer compatible.
    framebuf: Framebuffer<'a>,
    /// An array the width of one line of the display,
    /// used for fast rect fill operations.
    scanline: &'a mut [Rgb565; WIDTH],
    dma: Peri<'a, AnyChannel>,
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> Display<'a, DELAY> {
    pub fn new(
        interface: Fmc<'a>,
        rst: embassy_stm32::gpio::Output<'a>,
        te: ExtiInput<'a>,
        delay: DELAY,
        dma: Peri<'a, AnyChannel>,
        framebuf: &'static mut FramebufferArray,
        scanline: &'a mut [Rgb565; WIDTH],
    ) -> Self {
        Self {
            interface,
            rst,
            te,
            delay,
            framebuf: FrameBuf::new(framebuf, WIDTH, HEIGHT),
            scanline,
            dma,
        }
    }

    /*fn rgb565_to_bytes(pixel: Rgb565) -> [u8; 2] {
        embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(pixel)
    }
    fn rgb565_to_u16(pixel: Rgb565) -> [u16; 1] {
        [u16::from_ne_bytes(
            embedded_graphics_core::pixelcolor::raw::ToBytes::to_ne_bytes(pixel),
        )]
    }
    fn rgb666_to_bytes(pixel: Rgb666) -> [u8; 3] {
        [pixel.r(), pixel.g(), pixel.b()].map(|x| x << 2)
    } */

    /// Sets the address window in the display GRAM where any data
    /// subsequent written will be appended to.
    ///
    /// Setting the address window is very important for rotated displays,
    /// otherwise the controller still assumes a non-rotated default window.
    pub fn set_address_window(&mut self) {
        // Set column address window
        let mut column_buffer = [0u8; 4];
        column_buffer[0..2].copy_from_slice(&0_u16.to_be_bytes());
        column_buffer[2..4].copy_from_slice(&(WIDTH as u16).to_be_bytes());
        self.write_raw_command(0x2A, &column_buffer);

        // Set page address window
        let mut row_buffer = [0u8; 4];
        row_buffer[0..2].copy_from_slice(&0_u16.to_be_bytes());
        row_buffer[2..4].copy_from_slice(&(HEIGHT as u16).to_be_bytes());
        self.write_raw_command(0x2B, &row_buffer);
    }

    /// Pushes the buffer to the display, using the CPU.
    ///
    /// DMA transfer methods should always be preferred.
    pub fn push_buffer(&mut self) {
        self.set_address_window();

        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]);

        // Iterates over each pixel and writes it to the memory
        // address mapped to the display
        for pixel in self.framebuf.into_iter() {
            self.interface.write_data(pixel.1.into_storage());
        }
    }

    /// Read the current scanline from the display controller.
    ///
    /// This can be used for syncronizing writing the display
    /// framebuffer to the display in tune with the current
    /// scanline position.
    pub async fn read_scanline(&mut self) -> u16 {
        let scanlines = self.read_command::<2>(0x45).await;

        u16::from_be_bytes([scanlines[0], scanlines[1]])
    }

    /// Pushes the buffer to the display using DMA.
    pub async fn push_buffer_dma(&mut self) -> Result<(), ()> {
        // Wait for the next frame start signal from the ILI9341.
        //
        // When the signal is HIGH, then the controller is not
        // updating the display panel contents from memory.
        self.te.wait_for_rising_edge().await;

        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]);

        // TODO: make some change to adjust the exhaustive options.
        let mut transfer_options = dma::TransferOptions::default();
        transfer_options.pburst = Burst::Single;
        transfer_options.mburst = Burst::Single;
        transfer_options.flow_ctrl = dma::FlowControl::Dma; // DMS is required with memory-to-memory
        // FIFO is 4-words, so can fit8 half-word (16 bit) values.
        //
        // FIFO is required in memory-to-memory mode.
        transfer_options.fifo_threshold = Some(FifoThreshold::Full);

        // Now that the display knows we're about to write a page of data,
        // we can start a memory-to-memory DMA transfer to offload the
        // framebuffer transfer from the CPU, allowing the CPU to continue
        // processing other things (i.e. audio).
        unsafe {
            // STM32's DMA NDTR register is only u16, so one
            // transfer can only do a maximum of 65535 items,
            // requiring us to split into two transfers.

            dma::Transfer::new_transfer_raw(
                self.dma.reborrow(),
                // Rgb565's underlying storage type is u16, so
                // we can directly cast it as a u16 pointer.
                self.framebuf.data.as_ptr() as *const u16,
                self.framebuf.data.len() / 2,
                // Write to the data address of the display.
                //
                // TODO: this address should be taken dynamically
                // from the FMC SRAM layer and not hard-coded.
                self.interface.data_addr(),
                transfer_options,
            )
            .await;

            dma::Transfer::new_transfer_raw(
                self.dma.reborrow(),
                // Rgb565's underlying storage type is u16, so
                // we can directly cast it as a u16 pointer.
                (self.framebuf.data.as_ptr() as *const u16).add(self.framebuf.data.len() / 2),
                self.framebuf.data.len() / 2,
                // Write to the data address of the display.
                //
                // TODO: this address should be taken dynamically
                // from the FMC SRAM layer and not hard-coded.
                self.interface.data_addr(),
                transfer_options,
            )
            .await;
        }

        Ok(())
        /*
                let dma = pac::DMA2; // TODO: get pointer
                let channel = dma.st(4); // channel 5

                // Number of transfers in target word size.
                let ndtr = self.framebuf.data.len();
                assert!(ndtr > 0 && ndtr <= 0xFFFF);

                // Source address
                ch.par().write_value(self.framebuf.data.as_ptr());
                // Destination address
                ch.m0ar().write_value(0x6000_0000); // TODO: Get from FMC instance
                // Number of data
                ch.ndtr().write_value(pac::dma::regs::Ndtr(ndtr as _));
                // Configuration register
                ch.cr().write(|w| {
                    w.set_dir(2); // b10 memory-to-memory
                    w.set_msize(1); // Half-word 16-bit
                    w.set_psize(1); // Half-word 16-bit
                    w.set_pl(1); // medium priority
                    w.set_minc(1); // increment source memory address after transfer
                    w.set_pinc(0); // don't increment peripheral address after transfer
                    w.set_teie(false); // transfer error interrupt enable
                    w.set_htie(false); // half transfer interrupt enable TODO: use for queuing up next frame later?
                    w.set_tcie(false); // transfer complete interrupt enable
                    w.set_circ(false); // disable circular mode
                    #[cfg(dma_v1)]
                    w.set_trbuff(true);
                    #[cfg(dma_v2)]
                    w.set_chsel(_request);
                    w.set_pburst(0); // single transfer
                    w.set_mburst(0); // signal transfer
                    w.set_pfctrl(0); // 0 for mem-to-mem
                    w.set_en(false); // don't start yet
                });
        */
        // let req: u8 = dma.request();

        // unsafe {
        //     dma::dma_bdma::Transfer::new_write(
        //         self.dma,
        //         req,
        //         duty,
        //         self.inner.regs_gp16().ccr(cc_channel.index()).as_ptr() as *mut u16,
        //         dma::TransferOptions {
        //             fifo_threshold: Some(FifoThreshold::Full),
        //             mburst: Burst::Incr8,
        //             ..Default::default()
        //         },
        //     )
        //     .await
        // }
    }

    // TODO: DMA
    pub fn write_raw_command(&mut self, cmd: u8, args: &[u8]) {
        // , args: &[u8]
        trace!("display command: 0x{:x} {=[u8]:x}", cmd, args);

        // Write to the lower byte so A0=0 and triggers command mode.
        self.interface.write_command(cmd, args); // as u16
    }

    pub async fn read_command<const N: usize>(&mut self, cmd: u8) -> [u8; N] {
        self.interface.write_command(cmd, &[]);

        // First return value is always garbage.
        let _ = self.interface.read_data();

        // Most read commands have more then one result.
        let mut results = [0u8; N];
        for i in 0..N {
            // Read the response.
            //
            // Note that commands and their args/returns are
            // all 8-bit, even when using a 16-bit bus.
            results[i] = self.interface.read_data() as u8;

            trace!(
                "display command read: {}:0x{:x}=0x{:x}({:#08b})",
                i, cmd, results[i], results[i]
            );
        }

        results
    }

    pub fn display_size(&self) -> (u32, u32) {
        (240, 320)
    }

    pub async fn init(&mut self) {
        // Before and during delay timings from ILI9341 datasheet.
        self.delay.delay_ms(5).await;
        self.rst.set_low();
        self.delay.delay_ms(10).await;
        self.rst.set_high();

        // Initial wait required after power on reset.
        self.delay.delay_ms(120).await;

        // Power control B
        // 00000000
        // 100 power control[3:4] 001
        // 001 DC_ena 0000
        self.write_raw_command(
            0xCF,
            &[
                0x00,       // Padding
                0b11000011, // C3 // Default=10100010
                // ESD protection enabled
                0b00110000, // 30 // Default=11110000
            ],
        );

        // Power on sequence control
        self.write_raw_command(
            0xED,
            &[
                // Soft start control
                0x64, // Power on sequence control
                0x03, 0x12, // DDVDH Enhance Mode
                0x81,
            ],
        );

        // Driver timing control A
        // TODO: different in https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(
            0xE8,
            &[
                // Gate driver non-overlapping timing control
                0x85, // EQ timing control
                0x10, // Pre-charge timing control
                0x79,
            ],
        );

        // Power control A
        self.write_raw_command(
            0xCB,
            &[
                0x39, 0x2C, 0x00, // vCore Control
                0x34, // DDVDH control
                0x02,
            ],
        );

        //// Pump ratio control
        self.write_raw_command(
            0xF7,
            &[
                // Ratio Control
                0x20,
            ],
        );

        // Driver timing control B
        self.write_raw_command(
            0xEA,
            &[
                // Gate driver timing control
                0x00, 0x00,
            ],
        );

        // Power control  VRH[5:0]
        self.write_raw_command(
            0xC0,
            &[
                0x22, // GVDD level, reference for VCOM and grayscale voltage.
            ],
        );

        // Power control SAP[2:0];BT[3:0]
        self.write_raw_command(
            0xC1,
            &[
                0x11, // Step-up circuit factor
            ],
        );

        // VCM control
        self.write_raw_command(
            0xC5,
            &[
                0x3a, // VCOMH Voltage
                0x1c, // VCOML Voltage
            ],
        );

        //VCM control2
        self.write_raw_command(
            0xC7,
            &[
                0xa9, // nVM and VCOM offset voltage
            ],
        );

        // Memory Access Control
        self.write_raw_command(
            0x36,
            &[
                // [MY, MX, MV, ML, BGR, MH, 0, 0]
                // Invert MX and MY to rotate 180
                // MV rotates 90
                // 0b00000000,
                0b00001100, // [3] RGB to internal BGR swap
            ],
        ); // 0x08

        self.write_raw_command(
            0x3A,
            &[
                //  DBI[2:0] = 101 for 6-bit pixel RGB565 color.
                // [0, DPI[2:0], 0, DBI[2:0]]
                0b01010101,
            ],
        );

        //  Frame control in normal mode
        self.write_raw_command(
            0xB1,
            &[
                // DIVA[1:0]
                0b00000000, // RTAN[4:0]
                0b00011011, // 70hz
            ],
        );

        // Display Function Control
        // TODO: very different from https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(
            0xB6,
            &[
                // TODO: document..
                0x0A, 0xA2,
            ],
        );

        // Interface control/MADCTL
        // Memory overflow, endianess, RGB interface control
        // TODO: is set for 16-bit color?
        self.write_raw_command(
            0xF6,
            &[
                // TFT_MAD_MX TFT_MAD_MY TFT_MAD_MV TFT_MAD_BGR
                0b00000000, // 0x40 | 0x08,
                0b00011101,
            ],
        );
        // 3Gamma Function Disable
        self.write_raw_command(0xF2, &[0x00]);

        //Gamma curve selected
        self.write_raw_command(
            0x26,
            &[
                0x01, // 0x01
            ],
        );

        // Set Gamma
        self.write_raw_command(
            0xE0,
            &[
                0x0F, 0x3F, 0x2F, 0x0C, 0x10, 0x0A, 0x53, 0xD5, 0x40, 0x0A, 0x13, 0x03, 0x08, 0x03,
                0x00,
            ],
        );

        //Set Gamma
        self.write_raw_command(
            0xE1,
            &[
                0x00, 0x00, 0x10, 0x03, 0x0F, 0x05, 0x2C, 0xA2, 0x3F, 0x05, 0x0E, 0x0C, 0x37, 0x3C,
                0x0F,
            ],
        );

        // Display inversion on
        self.write_raw_command(0x21, &[]);

        // Tearing effect out enable
        // Remove when done testing.
        self.write_raw_command(0x35, &[0b00000001]);

        // Exit Sleep
        //
        // Delay required after because controller loads manufacture settings
        // and register values after the sleep exit command is called.
        self.write_raw_command(0x11, &[]);
        self.delay.delay_ms(120).await;

        // Display on
        //
        // Exits out of display-off mode and starts reading from frame memory.
        self.write_raw_command(0x29, &[]);
        self.delay.delay_ms(50).await;

        // Since we're drawing to the entire display,
        // we can set the address window once and
        // not need to update it again.
        //
        // This also "fixes" the address pointers if
        // the display has been rotated, otherwise
        // there is only a partial display render.
        self.set_address_window();

        // // Read display identification.
        // // 1st response is dummy data
        // // 2nd response is LCD module's manufacturer
        // // 3rd response is module/driver version
        // // 4th response is LCD module/driver ID
        // let ident: [u8; 4] = self.read_command(0x04);

        // // Read display status.
        // let status: [u8; 5] = self.read_command(0x09);
    }
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> OriginDimensions for Display<'a, DELAY> {
    fn size(&self) -> embedded_graphics::prelude::Size {
        let (width, height) = (WIDTH as u32, HEIGHT as u32);
        Size::new(width, height)
    }
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> DrawTarget for Display<'a, DELAY> {
    type Color = Rgb565;

    type Error = <Framebuffer<'a> as DrawTarget>::Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), <Framebuffer<'a> as DrawTarget>::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        self.framebuf.draw_iter(pixels)
    }

    /// Fill a given area with a solid color.
    ///
    /// This overrides the default DrawTarget implementation
    /// to do an accelerated fill on the target area.
    ///
    /// see: https://joshondesign.com/2022/09/29/make_rects_fast_rust
    fn fill_solid(
        &mut self,
        area: &Rectangle,
        color: <Framebuffer<'a> as DrawTarget>::Color,
    ) -> Result<(), <Framebuffer<'a> as DrawTarget>::Error> {
        // TODO: should probably check that we're within bounds...

        // Break our scanline down into the width of the target
        // rectangle and fill it with the intended color.
        //
        // TODO: we can probably get rid of this seconard scanline
        // struct by using the first line of the fill area in the
        // framebuffer memory.
        let slice = &mut self.scanline[0..area.size.width as usize];
        slice.fill(color);

        // Loop over each row in the framebuffer as a chunk.
        for (j, row_slice) in self
            .framebuf
            .data
            .chunks_exact_mut(WIDTH as usize)
            .enumerate()
        {
            let j = j as i32;

            // Check that we're below the area to be filled.
            if j < area.top_left.y {
                continue;
            }

            // Check that we're above the lower bound..
            if j >= area.top_left.y + area.size.height as i32 {
                continue;
            }

            let (_, after) = row_slice.split_at_mut((area.top_left.x) as usize);
            let (middle, _) = after.split_at_mut((area.size.width as usize) as usize);
            middle.copy_from_slice(&slice);
        }

        Ok(())
    }

    /// Fill the entire display with a solid color.
    ///
    /// Override the default implementation to use a more
    /// efficient method of filling the framebuffer instead
    /// of the default of iterating over every pixel.
    ///
    /// The default implementation of this method delegates to [`fill_solid`] to fill the
    /// [`bounding_box`] returned by the [`Dimensions`] implementation.
    ///
    /// [`Dimensions`]: super::geometry::Dimensions
    /// [`bounding_box`]: super::geometry::Dimensions::bounding_box
    /// [`fill_solid`]: DrawTarget::fill_solid()
    ///
    /// see: https://joshondesign.com/2022/09/29/make_rects_fast_rust
    fn clear(&mut self, color: Self::Color) -> Result<(), <Framebuffer<'a> as DrawTarget>::Error> {
        // Fill the framebuffer memory directly.
        // self.framebuf.data.fill(color);
        // TODO: try with DMA2D

        self.scanline.fill(color);

        // Loop over each row in the framebuffer as a chunk.
        //
        // We benchmarked this as 31x faster then .fill on the framebuffer directly!
        //
        // see: https://joshondesign.com/2022/09/29/make_rects_fast_rust
        for (j, row_slice) in self
            .framebuf
            .data
            .chunks_exact_mut(WIDTH as usize)
            .enumerate()
        {
            row_slice.copy_from_slice(self.scanline);
        }

        Ok(())
    }
}
