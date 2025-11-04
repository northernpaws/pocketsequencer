// TODO: integrate https://github.com/okhsunrog/lcd-async/tree/master to add async

pub mod interface;

use core::convert::Infallible;
use core::ptr::{self, null};
use defmt::trace;

use embassy_executor::Spawner;
use embassy_stm32::dma::{self, AnyChannel};
use embassy_stm32::fmc::Fmc;
use embassy_stm32::peripherals::FMC;
use embassy_time::Delay;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::IntoStorage;
use embedded_io_async::Read;
use static_cell::StaticCell;

use embedded_graphics_framebuf::FrameBuf;

use crate::hardware::display;
use crate::hardware::display::interface::{
    Interface,
    ReadInterface,
    WriteInterface
};

pub const WIDTH: usize = 240;
pub const HEIGHT: usize = 320;

pub struct Display<'a,
    DELAY: embedded_hal_async::delay::DelayNs,
    I: Interface
> {
    interface: I,
    rst: embassy_stm32::gpio::Output<'a>,
    delay: DELAY,
    /// Embedded-graphics compatible DrawTarget framebuffer for updating the display.
    /// 
    /// A framebuffer is used instead of drawing to the display directly
    /// to allow full-frame updates that are DMA transfer compatible.
    framebuf: FrameBuf<Rgb565, [Rgb565; WIDTH * HEIGHT]>,
    dma: Peri<'a, AnyChannel>
}

impl<'a,
    DELAY: embedded_hal_async::delay::DelayNs,
    I: Interface + WriteInterface
> Display<'a, DELAY, I> {
    pub fn new (
        interface: I,
        rst: embassy_stm32::gpio::Output<'a>,
        delay: DELAY,
        framebuffer: &'a mut [Rgb565; WIDTH * HEIGHT],
        dma: Peri<'a, AnyChannel>
    ) -> Self {
        Self {
            interface,
            rst,
            delay,
            framebuf: FrameBuf::new([Rgb565; WIDTH * HEIGHT], WIDTH, HEIGHT),
            dma
        }
    }

    /// Returns a reference to the display's framebuffer.
    /// 
    /// This is an embedded-graphics DrawTarget that can be
    /// used to updated the next frame for the display.
    pub fn framebuffer_ref<'b>(&mut self) -> &'b mut FrameBuf<Rgb565, [Rgb565; WIDTH * HEIGHT]> {
        &mut self.framebuf
    }

    /// Pushes the buffer to the display, using the CPU.
    /// 
    /// DMA transfer methods should always be preferred.
    pub async fn push_buffer(&mut self) -> Result<(), <I as Interface>::Error> {
        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]).await?;

        for pixel in self.framebuf.into_iter() {
            self.interface.write_data(pixel.1.into_storage());
        }

        Ok(())
    }

    /// Pushes the buffer to the display using DMA.
    pub async fn push_buffer_dma (&mut self) {
        // First we need to send a memory addressing command to tell
        // the display where we're starting to write data from.
        //
        // 0x2C is a Memory Write command, when executed it resets the
        // column and page counters to the start of the memory and
        // subsequent writes increment the counters as data comes in.
        self.write_raw_command(0x2C, &[]).await?;

        let req = self.dma.request();

        // Now that the display knows we're about to write a page of data,
        // we can start a memory-to-memory DMA transfer to offload the
        // framebuffer transfer from the CPU, allowing the CPU to continue
        // processing other things (i.e. audio).
        unsafe {
            let transfer = dma::Transfer::new_transfer_raw(
                dma,
                req,
                // Rgb565's underlying storage type is u16, so
                // we can directly cast it as a u16 pointer.
                self.framebuf.data.as_ptr() as *const u16,
                self.framebuf.data.len(),
                // Write to the data address of the display.
                //
                // TODO: this address should be taken dynamically
                // from the FMC SRAM layer and not hard-coded.
                0x6000_0001 as *mut _,
                dma::TransferOptions {
                    pburst: Burst::Single,
                    mburst: Burst::Single,
                    // We need to set an FIFO threshold because non-FIFO "direct
                    // mode" is not supported for memory-to-memory transfers.
                    fifo_threshold: Some(FifoThreshold::Full),
                    ..Default::default()
                },
            ).await;
        }
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
    pub async fn write_raw_command(&mut self, cmd: u8, args: &[u8]) -> Result<(), I::Error> { // , args: &[u8]
        trace!("display command: 0x{:x} {=[u8]:x}", cmd, args);

        // Write to the lower byte so A0=0 and triggers command mode.
        self.interface.write_command(cmd, args).await?; // as u16

        Ok(())
    }

    pub fn display_size(&self) -> (u32, u32) {
        (240,320)
    }

    pub async fn init (&mut self) -> Result<(), I::Error> {
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
        self.write_raw_command(0xCF, &[
            0x00, // Padding
            0b11000011, // C3 // Default=10100010
            // ESD protection enabled
            0b00110000  // 30 // Default=11110000
        ]).await?;

        // Power on sequence control
        self.write_raw_command(0xED, &[
            // Soft start control
            0x64,
            // Power on sequence control
            0x03,
            0x12,
            // DDVDH Enhance Mode
            0x81
        ]).await?; 

        // Driver timing control A
        // TODO: different in https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(0xE8, &[
            // Gate driver non-overlapping timing control
            0x85,
            // EQ timing control
            0x10,
            // Pre-charge timing control
            0x79,
        ]).await?;

        // Power control A
        self.write_raw_command(0xCB, &[
            0x39,
            0x2C,
            0x00,
            // vCore Control
            0x34,
            // DDVDH control
            0x02,
        ]).await?;

        //// Pump ratio control
        self.write_raw_command(0xF7, &[
            // Ratio Control
            0x20
        ]).await?;

        // Driver timing control B
        self.write_raw_command(0xEA, &[
            // Gate driver timing control
            0x00,
            0x00
        ]).await?;

        // Power control  VRH[5:0] 
        self.write_raw_command(0xC0, &[
            0x22 // GVDD level, reference for VCOM and grayscale voltage.
        ]).await?;

        // Power control SAP[2:0];BT[3:0] 
        self.write_raw_command(0xC1, &[
            0x11 // Step-up circuit factor
        ]).await?;

        //VCM control
        self.write_raw_command(0xC5, &[
            0x3a, // VCOMH Voltage
            0x1c // VCOML Voltage
        ]).await?;

        //VCM control2 
        self.write_raw_command(0xC7, &[
            0xa9 // nVM and VCOM offset voltage
        ]).await?;

        // Memory Access Control 
        self.write_raw_command(0x36, &[
            // [MY, MX, MV, ML, BGR, MH, 0, 0]
            0b00000000
        ]).await?; // 0x08
        self.write_raw_command(0x3A, &[
            //  DBI[2:0] = 101 for 6-bit pixel RGB565 color.

            // [0, DPI[2:0], 0, DBI[2:0]]
            0b01010101
        ]).await?;

        //  Frame control in normal mode
        self.write_raw_command(0xB1, &[
            // DIVA[1:0]
            0b00000000,
            // RTAN[4:0]
            0b00011011 // 70hz
        ]).await?;

        // Display Function Control 
        // TODO: very different from https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(0xB6, &[
            // TODO: document..
            0x0A,
            0xA2
        ]).await?;
        
        // Interface control/MADCTL
        // Memory overflow, endianess, RGB interface control
        // TODO: is set for 16-bit color?
        self.write_raw_command(0xF6, &[
            0x01,
            0x1d
        ]).await?;

        // 3Gamma Function Disable 
        self.write_raw_command(0xF2, &[
            0x00
        ]).await?;

        //Gamma curve selected 
        self.write_raw_command(0x26, &[
            0x01, // 0x01
        ]).await?;

        // Set Gamma 
        self.write_raw_command(0xE0, &[0x0F,0x3F,0x2F,0x0C,0x10,0x0A,0x53,0xD5,0x40,0x0A,0x13,0x03,0x08,0x03,0x00]).await?;

        //Set Gamma 
        self.write_raw_command(0xE1, &[0x00,0x00,0x10,0x03,0x0F,0x05,0x2C,0xA2,0x3F,0x05,0x0E,0x0C,0x37,0x3C,0x0F]).await?;
        
        // Display inversion on  
        self.write_raw_command(0x21, &[]).await?;

        // Tearing effect out enable
        // Remove when done testing.
        self.write_raw_command(0x35, &[0b00000001]).await?;

        // Exit Sleep 
        //
        // Delay required after because controller loads manufacture settings
        // and register values after the sleep exit command is called.
        self.write_raw_command(0x11, &[]).await?;
        self.delay.delay_ms(120).await;

        // Display on 
        //
        // Exits out of display-off mode and starts reading from frame memory.
        self.write_raw_command(0x29, &[]).await?;
        self.delay.delay_ms(50).await;

        // // Read display identification.
        // // 1st response is dummy data
        // // 2nd response is LCD module's manufacturer
        // // 3rd response is module/driver version
        // // 4th response is LCD module/driver ID
        // let ident: [u8; 4] = self.read_command(0x04);

        // // Read display status.
        // let status: [u8; 5] = self.read_command(0x09);

        Ok(())
    }
}

impl<'a,
    DELAY: embedded_hal_async::delay::DelayNs,
    I: Interface + WriteInterface
> Display<'a, DELAY, I> 
where
    I: Interface + WriteInterface + ReadInterface,
    I::Word: Into<u8> + Eq,
{
    pub async fn read_command<const N: usize>(&mut self, cmd: u8) -> Result<[u8; N], I::Error> {
        self.interface.write_command(cmd, &[]).await?;

        // Most read commands have more then one result.
        let mut results = [0u8; N];
        for i in 0..N {
            // Read the response.
            //
            // Note that commands and their args/returns are
            // all 8-bit, even when using a 16-bit bus.
            //
            // TODO: some commands have multiple response bytes.
            results[i] = self.interface.read_data().await?.into(); // Uses num::cast::AsPrimitive for cast

            trace!("display command read: {}:0x{:x}=0x{:x}({:#08b})", i, cmd, results[i], results[i]);
        }

        Ok(results)
    }
}
