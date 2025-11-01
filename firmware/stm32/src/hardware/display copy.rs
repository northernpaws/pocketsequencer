
use embassy_executor::Spawner;
use embassy_time::Delay;
use lcd_async::dcs::{self, InterfaceExt};
use static_cell::StaticCell;

use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};

use lcd_async::{
    interface::{
        ParallelInterface,
        Generic16BitBus
    },
    models::ILI9341Rgb565,
    Builder
};

use lcd_async::raw_framebuf::RawFrameBuf;

const WIDTH: usize = 240;
const HEIGHT: usize = 320;
// Rgb565 uses 2 bytes per pixel
const FRAME_BUFFER_SIZE: usize = WIDTH * HEIGHT * 2;

static FRAME_BUFFER: StaticCell<[u8; FRAME_BUFFER_SIZE]> = StaticCell::new();

pub struct Display<'a, DELAY: embedded_hal_async::delay::DelayNs> {
    delay: DELAY,
    display: lcd_async::Display<
        ParallelInterface<
            Generic16BitBus<
                embassy_stm32::gpio::Output<'a>, // d0
                embassy_stm32::gpio::Output<'a>, // d1
                embassy_stm32::gpio::Output<'a>, // d2
                embassy_stm32::gpio::Output<'a>, // d3
                embassy_stm32::gpio::Output<'a>, // d4
                embassy_stm32::gpio::Output<'a>, // d5
                embassy_stm32::gpio::Output<'a>, // d6
                embassy_stm32::gpio::Output<'a>, // d7
                embassy_stm32::gpio::Output<'a>, // d8
                embassy_stm32::gpio::Output<'a>, // d9
                embassy_stm32::gpio::Output<'a>, // d10
                embassy_stm32::gpio::Output<'a>, // d11
                embassy_stm32::gpio::Output<'a>, // d12
                embassy_stm32::gpio::Output<'a>, // d13
                embassy_stm32::gpio::Output<'a>, // d14
                embassy_stm32::gpio::Output<'a>  // d15
            >,
            embassy_stm32::gpio::Output<'a>, // data/command
            embassy_stm32::gpio::Output<'a>  // write enable
        >,
        ILI9341Rgb565,
        embassy_stm32::gpio::Output<'a>>,
        frame_buffer: &'a mut [u8; FRAME_BUFFER_SIZE]
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> Display<'a, DELAY> {
    pub async fn new (
        mut rst: embassy_stm32::gpio::Output<'a>,
        wr: embassy_stm32::gpio::Output<'a>,
        dc: embassy_stm32::gpio::Output<'a>,
        d0: embassy_stm32::gpio::Output<'a>,
        d1: embassy_stm32::gpio::Output<'a>,
        d2: embassy_stm32::gpio::Output<'a>,
        d3: embassy_stm32::gpio::Output<'a>,
        d4: embassy_stm32::gpio::Output<'a>,
        d5: embassy_stm32::gpio::Output<'a>,
        d6: embassy_stm32::gpio::Output<'a>,
        d7: embassy_stm32::gpio::Output<'a>,
        d8: embassy_stm32::gpio::Output<'a>,
        d9: embassy_stm32::gpio::Output<'a>,
        d10: embassy_stm32::gpio::Output<'a>,
        d11: embassy_stm32::gpio::Output<'a>,
        d12: embassy_stm32::gpio::Output<'a>,
        d13: embassy_stm32::gpio::Output<'a>,
        d14: embassy_stm32::gpio::Output<'a>,
        d15: embassy_stm32::gpio::Output<'a>,
        mut delay: DELAY,
    ) -> Self {
        // Create a 16-bit data bus.
        let mut bus = Generic16BitBus::new((
            d0, d1, d2, d3, d4, d5, d6, d7, d8,
            d9, d10, d11, d12, d13, d14, d15,
        ));

        // Create the parallel display interface using the previously
        // created bus and Data/Command and write enable pins.
        let mut interface = ParallelInterface::new(bus, dc, wr);
        
        // Hardware reset
        rst.set_low();
        delay.delay_us(10).await;
        rst.set_high();

        // Software reset just in case?
        interface
                .write_command(dcs::SoftReset)
                .await.unwrap();
            
        // 15.4:  It is necessary to wait 5msec after releasing RESX before sending commands.
        // 8.2.2: It will be necessary to wait 5msec before sending new command following software reset.
        delay.delay_us(5_000).await;
        
        // Power control B
        interface.write_raw(0xCF, &[0x00, 0xC1, 0x30]).await.unwrap();
        // Power on sequence control
        interface.write_raw(0xED, &[0x64, 0x03, 0x12, 0x81]).await.unwrap();
        // Driver timing control A
        interface.write_raw(0xE8, &[0x85, 0x00, 0x78]).await.unwrap();
        // Power control A
        interface.write_raw(0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02]).await.unwrap();
        // Pump ratio control
        interface.write_raw(0xF7, &[0x20]).await.unwrap();
        // Drvier timing control B
        interface.write_raw(0xEA, &[0x00, 0x00]).await.unwrap();

        // Power control 1 VRH[5:0]
        interface.write_raw(0xC0, &[0x23]).await.unwrap();
        // Power control 2 SAP[2:0];BT[3:0]  
        interface.write_raw(0xC1, &[0x10]).await.unwrap();

        // VCM control 1
        interface.write_raw(0xC5, &[0x3e, 0x28]).await.unwrap();
        // VCM control 2
        interface.write_raw(0xC7, &[0x86]).await.unwrap();

        // MADCTRL ILI9341_MADCTL_MX | 
        interface.write_raw(0x36, &[( 0x40 | 0x08)]).await.unwrap();
        // Pixel format
        interface.write_raw(0x3A, &[0x55]).await.unwrap();
        // FRMCTRL1?
        interface.write_raw(0xB1, &[0x00, 0x18]).await.unwrap();
        // ILI9341_DFUNCTR
        interface.write_raw(0xB6, &[0x08, 0x82, 0x27]).await.unwrap();
        // Gamma disable
        interface.write_raw(0xF2, &[0x00]).await.unwrap();
        // Gamma curve select
        interface.write_raw(0x26, &[0x01]).await.unwrap();
        // Set gamma
        interface.write_raw(0xE0, &[0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00]).await.unwrap();
        interface.write_raw(0xE1, &[0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F]).await.unwrap();

        // ILI9341_SLPOUT
        interface.write_raw(0x11, &[120]).await.unwrap();

        delay.delay_ns(5_000);

        // Display on
        interface.write_raw(0x29, &[]).await.unwrap();

        let mut display = Builder::new(ILI9341Rgb565, interface) // Using ST7789 as an example model
            .reset_pin(rst)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .init(&mut delay)
            .await
            .unwrap();
            // NOTE: we don't call .init() as we handle to init sequence ourselves.

        let frame_buffer = FRAME_BUFFER.init([0; FRAME_BUFFER_SIZE]);

        // Create a framebuffer for drawing
        let mut raw_fb =
            RawFrameBuf::<Rgb565, _>::new(frame_buffer.as_mut_slice(), WIDTH.into(), HEIGHT.into());

        // Clear the framebuffer to black
        raw_fb.clear(Rgb565::BLACK).unwrap();

        // Send the framebuffer data to the display
        display.show_raw_data(
            0, 0,
            WIDTH as u16, HEIGHT as u16,
                frame_buffer
        ).await.unwrap();
            
        // // 5. Create a framebuffer in a new scope to draw the scene.
        // {
        //     let mut fbuf = RawFrameBuf::<Rgb565, _>::new(frame_buffer, WIDTH, HEIGHT);

        //     // Draw anything from `embedded-graphics` into the in-memory buffer.
        //     fbuf.clear(Rgb565::BLACK).unwrap();
        //     Circle::new(Point::new(120, 120), 80)
        //         .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        //         .draw(&mut fbuf)
        //         .unwrap();
        // } // `fbuf` is dropped here, releasing the mutable borrow.

        // // 6. Send the entire rendered frame to the display.
        // display
        //     .show_raw_data(0, 0, WIDTH as u16, HEIGHT as u16, frame_buffer)
        //     .await
        //     .unwrap();
        
        Self {
            delay,
            display,
            frame_buffer
        }
    }

}

