// TODO: integrate https://github.com/okhsunrog/lcd-async/tree/master to add async

pub mod sram;
pub mod interface;

use core::convert::Infallible;
use core::ptr::{self, null};
use defmt::trace;

use embassy_executor::Spawner;
use embassy_stm32::fmc::Fmc;
use embassy_stm32::peripherals::FMC;
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
use stm32_fmc::{FmcBank, modify_reg};

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
        mut delay: DELAY
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
            rd,
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

pub struct SramDisplay<'a,
    DELAY: embedded_hal_async::delay::DelayNs
> {
    instance: sram::Sram,
    rst: embassy_stm32::gpio::Output<'a>,
    delay: DELAY,
    memory: &'a mut [u16],
}

impl<'a,
    DELAY: embedded_hal_async::delay::DelayNs
> SramDisplay<'a, DELAY> {
    pub fn new (
        instance: sram::Sram,
        rst: embassy_stm32::gpio::Output<'a>,
        delay: DELAY,
    ) -> Self {
        let ram_slice: &mut [u16] = unsafe {
            // Get the memory address pointer.
            let ram_ptr: *mut u16 = instance.ptr() as *mut _;

            // Convert raw pointer to slice.
            core::slice::from_raw_parts_mut(ram_ptr, 2)
        };

        Self {
            instance,
            rst,
            delay,
            memory: ram_slice,
        }
    }

    pub fn read_command<const N: usize>(&mut self, cmd: u8) -> [u8; N] {
        // Write to the lower byte so A0=0 and triggers command mode.
        self.memory[0] = 0x00 as u16;

        // Most read commands have more then one result.
        let mut results = [0u8; N];
        for i in 0..N {
            // Read the response.
            //
            // Note that commands and their args/returns are
            // all 8-bit, even when using a 16-bit bus.
            //
            // TODO: some commands have multiple response bytes.
            results[i] = self.memory[1] as u8;

            trace!("display command read: {}:0x{:x}=0x{:x}({:#08b})", i, cmd, results[i], results[i]);
        }

        results
    }

    // TODO: DMA
    pub fn write_raw_command(&mut self, cmd: u8, args: &[u8]) { // , args: &[u8]
        trace!("display command: 0x{:x} {=[u8]:x}", cmd, args);

        // Write to the lower byte so A0=0 and triggers command mode.
        self.memory[0] = cmd as u16; // cmd as u16;

        // let ptr = self.memory.as_ptr();

        // Write to the upper byte so A0=1 triggering data mode.
        for i in 0..args.len() {
            self.memory[1] = args[i] as u16;
        }
    }

    pub async fn init (&mut self) -> Result<(), ()> {
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
        ]);

        // Power on sequence control
        self.write_raw_command(0xED, &[
            // Soft start control
            0x64,
            // Power on sequence control
            0x03,
            0x12,
            // DDVDH Enhance Mode
            0x81
        ]); 

        // Driver timing control A
        // TODO: different in https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(0xE8, &[
            // Gate driver non-overlapping timing control
            0x85,
            // EQ timing control
            0x10,
            // Pre-charge timing control
            0x79,
        ]);

        // Power control A
        self.write_raw_command(0xCB, &[
            0x39,
            0x2C,
            0x00,
            // vCore Control
            0x34,
            // DDVDH control
            0x02,
        ]);

        //// Pump ratio control
        self.write_raw_command(0xF7, &[
            // Ratio Control
            0x20
        ]);

        // Driver timing control B
        self.write_raw_command(0xEA, &[
            // Gate driver timing control
            0x00,
            0x00
        ]);

        // Power control  VRH[5:0] 
        self.write_raw_command(0xC0, &[
            0x22 // GVDD level, reference for VCOM and grayscale voltage.
        ]);

        // Power control SAP[2:0];BT[3:0] 
        self.write_raw_command(0xC1, &[
            0x11 // Step-up circuit factor
        ]);

        //VCM control
        self.write_raw_command(0xC5, &[
            0x3a, // VCOMH Voltage
            0x1c // VCOML Voltage
        ]);

        //VCM control2 
        self.write_raw_command(0xC7, &[
            0xa9 // nVM and VCOM offset voltage
        ]);

        // Memory Access Control 
        self.write_raw_command(0x36, &[
            // [MY, MX, MV, ML, BGR, MH, 0, 0]
            0b00000000
        ]); // 0x08
        self.write_raw_command(0x3A, &[
            //  DBI[2:0] = 101 for 6-bit pixel RGB565 color.

            // [0, DPI[2:0], 0, DBI[2:0]]
            0b01010101
        ]);

        //  Frame control in normal mode
        self.write_raw_command(0xB1, &[
            // DIVA[1:0]
            0b00000000,
            // RTAN[4:0]
            0b00011011 // 70hz
        ]);

        // Display Function Control 
        // TODO: very different from https://github.com/Bodmer/TFT_ILI9341/blob/master/TFT_ILI9341.cpp
        self.write_raw_command(0xB6, &[
            // TODO: document..
            0x0A,
            0xA2
        ]);
        
        // Interface control/MADCTL
        // Memory overflow, endianess, RGB interface control
        // TODO: is set for 16-bit color?
        self.write_raw_command(0xF6, &[
            0x01,
            0x1d
        ]);

        // 3Gamma Function Disable 
        self.write_raw_command(0xF2, &[
            0x00
        ]);

        //Gamma curve selected 
        self.write_raw_command(0x26, &[
            0x01, // 0x01
        ]);

        // Set Gamma 
        self.write_raw_command(0xE0, &[0x0F,0x3F,0x2F,0x0C,0x10,0x0A,0x53,0xD5,0x40,0x0A,0x13,0x03,0x08,0x03,0x00]);

        //Set Gamma 
        self.write_raw_command(0xE1, &[0x00,0x00,0x10,0x03,0x0F,0x05,0x2C,0xA2,0x3F,0x05,0x0E,0x0C,0x37,0x3C,0x0F]);
        
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

        // Read display identification.
        // 1st response is dummy data
        // 2nd response is LCD module's manufacturer
        // 3rd response is module/driver version
        // 4th response is LCD module/driver ID
        let ident: [u8; 4] = self.read_command(0x04);

        // Read display status.
        let status: [u8; 5] = self.read_command(0x09);

        Ok(())
    }
}