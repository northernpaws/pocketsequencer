// TODO: integrate https://github.com/okhsunrog/lcd-async/tree/master to add async

use core::convert::Infallible;
use defmt::trace;

use embassy_executor::Spawner;
use embassy_time::Delay;
use mipidsi::interface;
use mipidsi::models::Model;
use static_cell::StaticCell;

use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};

use mipidsi::{
    dcs::{self, InterfaceExt},
    interface::{
        ParallelInterface,
        Generic16BitBus,
        ParallelError
    },
    models::ILI9341Rgb565,
    Builder
};

const WIDTH: usize = 240;
const HEIGHT: usize = 320;

type ParallelInterface16<'a> = ParallelInterface<
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
        >;

// embedded_hal_async::delay::DelayNs 
pub struct Display<'a, DELAY: embedded_hal::delay::DelayNs> {
    delay: DELAY,
    display: mipidsi::Display<
        ParallelInterface16<'a>,
        ILI9341Rgb565,
        embassy_stm32::gpio::Output<'a>>,
    rd: embassy_stm32::gpio::Output<'a>
}

impl<'a, DELAY: embedded_hal::delay::DelayNs> Display<'a, DELAY> {
    pub fn new (
        mut rst: embassy_stm32::gpio::Output<'a>,
        wr: embassy_stm32::gpio::Output<'a>,
        dc: embassy_stm32::gpio::Output<'a>,
        mut rd: embassy_stm32::gpio::Output<'a>,
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
        // Test a register read
        
        // Ensure that RD is high.
        rd.set_high();

        // Hardware reset   
        // trace!("Display hardware reset");
        // rst.set_low();
        // delay.delay_us(10);
        // rst.set_high();

        // Create a 16-bit data bus.
        let mut bus = Generic16BitBus::new((
            d0, d1, d2, d3, d4, d5, d6, d7, d8,
            d9, d10, d11, d12, d13, d14, d15,
        ));

        // Create the parallel display interface using the previously
        // created bus and Data/Command and write enable pins.
        let mut interface = ParallelInterface::new(bus, dc, wr);

        /*delay.delay_ms(5);
        rst.set_low();
        delay.delay_ms(10);
        rst.set_high();
        delay.delay_ms(120);

        interface.write_raw(0xCF, &[0x00, 0xc3, 0x30]).unwrap();
        interface.write_raw(0xED, &[0x64, 0x03, 0x12, 0x81]).unwrap();
        interface.write_raw(0xE8, &[0x85, 0x10, 0x79]).unwrap();
        interface.write_raw(0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02]).unwrap();
        interface.write_raw(0xF7, &[0x20]).unwrap();
        interface.write_raw(0xEA, &[0x00, 0x00]).unwrap();
        // Power control  VRH[5:0] 
        interface.write_raw(0xC0, &[0x22]).unwrap();
        // Power control SAP[2:0];BT[3:0] 
        interface.write_raw(0xC1, &[0x11]).unwrap();
        //VCM control
        interface.write_raw(0xC5, &[0x3a, 0x1c]).unwrap();
        //VCM control2 
        interface.write_raw(0xC7, &[0xa9]).unwrap();
        // Memory Access Control 
        interface.write_raw(0x36, &[0x08]).unwrap();
        interface.write_raw(0x3A, &[0x55]).unwrap();
        //  //70hz 
        interface.write_raw(0xB1, &[0x00, 0x1B]).unwrap();
        // Display Function Control 
        interface.write_raw(0xB6, &[0x0A, 0xA2]).unwrap();
        
        interface.write_raw(0xF6, &[0x01, 0x1d]).unwrap();
        // 3Gamma Function Disable 
        interface.write_raw(0xF2, &[0x00]).unwrap();
        //Gamma curve selected 
        interface.write_raw(0x26, &[0x01]).unwrap();
        //Set Gamma 
        interface.write_raw(0xE0, &[0x0F,0x3F,0x2F,0x0C,0x10,0x0A,0x53,0xD5,0x40,0x0A,0x13,0x03,0x08,0x03,0x00]).unwrap();
        //Set Gamma 
        interface.write_raw(0xE1, &[0x00,0x00,0x10,0x03,0x0F,0x05,0x2C,0xA2,0x3F,0x05,0x0E,0x0C,0x37,0x3C,0x0F]).unwrap();
        
        //Display inversino on  
        interface.write_raw(0x21, &[]).unwrap();
        //Exit Sleep 
        interface.write_raw(0x11, &[]).unwrap();
        
        delay.delay_ms(120);
        //Display on 
        interface.write_raw(0x29, &[]).unwrap();
        delay.delay_ms(50);*/

        let mut display = Builder::new(ILI9341Rgb565, interface) // Using ST7789 as an example model
            .reset_pin(rst)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .init(&mut delay)
            .unwrap();
            // NOTE: we don't call .init() as we handle to init sequence ourselves.
        
        Self {
            delay,
            display,
            rd
        }
    }

    pub fn set_pixel(&mut self, x: u16, y: u16, color: <ILI9341Rgb565 as Model>::ColorFormat) -> Result<(), <ParallelInterface16<'a> as interface::Interface>::Error> {
        self.display.set_pixel(x, y, color)
    }

    pub fn clear(&mut self, color: Rgb565) -> Result<(), ParallelError<Infallible, Infallible, Infallible>> {
        self.rd.set_high();
        self.display.clear(color)
    }
}

