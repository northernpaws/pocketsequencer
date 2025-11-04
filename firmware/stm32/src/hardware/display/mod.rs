// TODO: integrate https://github.com/okhsunrog/lcd-async/tree/master to add async

pub mod fmc;

use defmt::trace;

use embassy_stm32::{
    Peri,
    dma::{self, AnyChannel},
};
use embedded_graphics::prelude::*;
use embedded_graphics::{pixelcolor::Rgb565, prelude::DrawTarget};
use embedded_graphics_core::pixelcolor::RgbColor;
use embedded_graphics_framebuf::FrameBuf;

use crate::hardware::display::fmc::Fmc;

pub const WIDTH: usize = 240;
pub const HEIGHT: usize = 320;

pub type Framebuffer = FrameBuf<Rgb565, [Rgb565; WIDTH * HEIGHT]>;

pub struct Display<'a, DELAY: embedded_hal_async::delay::DelayNs> {
    interface: Fmc,
    rst: embassy_stm32::gpio::Output<'a>,
    delay: DELAY,
    /// Embedded-graphics compatible DrawTarget framebuffer for updating the display.
    ///
    /// A framebuffer is used instead of drawing to the display directly
    /// to allow full-frame updates that are DMA transfer compatible.
    framebuf: Framebuffer,
    dma: Peri<'a, AnyChannel>,
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> Display<'a, DELAY> {
    pub fn new(
        interface: Fmc,
        rst: embassy_stm32::gpio::Output<'a>,
        delay: DELAY,
        dma: Peri<'a, AnyChannel>,
    ) -> Self {
        Self {
            interface,
            rst,
            delay,
            framebuf: FrameBuf::new([Rgb565::BLACK; WIDTH * HEIGHT], WIDTH, HEIGHT),
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

    /// Pushes the buffer to the display, using the CPU.
    ///
    /// DMA transfer methods should always be preferred.
    pub fn push_buffer(&mut self) {
        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]);

        for pixel in self.framebuf.into_iter() {
            self.interface.write_data(pixel.1.into_storage());
        }
    }

    /// Pushes the buffer to the display using DMA.
    pub async fn push_buffer_dma(&mut self) -> Result<(), ()> {
        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]);

        // TODO: make some change to adjust the exhaustive options.
        let transfer_options = dma::TransferOptions::default();
        // transfer_options.pburst = Burst::Single;
        // transfer_options.mburst = Burst::Single;
        // transfer_options.fifo_threshold = Some(FifoThreshold::Full);

        // Now that the display knows we're about to write a page of data,
        // we can start a memory-to-memory DMA transfer to offload the
        // framebuffer transfer from the CPU, allowing the CPU to continue
        // processing other things (i.e. audio).
        unsafe {
            let transfer = dma::Transfer::new_transfer_raw(
                self.dma.reborrow(),
                // Rgb565's underlying storage type is u16, so
                // we can directly cast it as a u16 pointer.
                self.framebuf.data.as_ptr() as *const u16,
                self.framebuf.data.len(),
                // Write to the data address of the display.
                //
                // TODO: this address should be taken dynamically
                // from the FMC SRAM layer and not hard-coded.
                0x6000_0001 as *mut u16,
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
    pub async fn write_raw_command(&mut self, cmd: u8, args: &[u8]) {
        // , args: &[u8]
        trace!("display command: 0x{:x} {=[u8]:x}", cmd, args);

        // Write to the lower byte so A0=0 and triggers command mode.
        self.interface.write_command(cmd, args); // as u16
    }

    pub async fn read_command<const N: usize>(&mut self, cmd: u8) -> [u8; N] {
        self.interface.write_command(cmd, &[]);

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

        //VCM control
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
                0b00000000,
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
        self.write_raw_command(0xF6, &[0x01, 0x1d]);

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

    type Error = <Framebuffer as DrawTarget>::Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), <Framebuffer as DrawTarget>::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        self.framebuf.draw_iter(pixels)
    }
}
