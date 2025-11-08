# v0.1

## Components

  * [480 MHz Arm® Cortex®-M7 32-bit Processor (`STM32H743ZIT6`)](https://www.st.com/en/microcontrollers-microprocessors/stm32h743zi.html)
    * Floating-point unit
    * DSP instructions
    * High-speed memory controller
    * Quad SPI interface for flash
    * High-speed USB host and device mode
    * Several SAI audio blocks
  * [24 bit Audio Codec (`NAU88C22YG`)](https://www.nuvoton.com/products/smart-home-audio/audio-converters/audio-codec-series/nau88c22yg/)
    * DAC: 94dB SNR and -84dB THD (“A” weighted)
    * ADC: 90dB SNR and -80dB THD (“A” weighted)
    * Integrated BTL speaker driver: 1W into 8Ω
    * Integrated head-phone driver: 40mW into 16Ω
  * [Optional Bluetooth Audio & MIDI (`nRF5340`)](https://www.nordicsemi.com/Products/nRF5340)
    * Wirelessly connect Bluethooth headphones and speakers
    * Connect devices for BLE MIDI inputs and outputs
  * [Optional Wi-Fi (`nRF7002`)](https://www.nordicsemi.com/Products/nRF7002)
    * Wi-Fi sample management and transfer

## Observations

### Routing

  - In next revision, use larger STM32 package. Ran out of pins on the left side due to the FMC and SDRAM, and had to route several GPIO across the entire board, including some interrupt traces in the power layer. Ran out of ADC pins for the velocity pads and had to add an $8 IO expander.

## Errata

 - QUADSPI CS should be NSS capable pin for automatic hardware peripheral control. Use PG6 and swap USB_INT to PD6.
 - Micro SD Card detect pins weren't connected, card detection needs to happen in software.
 - SPI1 hardward CS pins should be PA4, PA15, or PG10. Currently using PA4 (SD CARD) and PC13 (internal flash).
 - Unused key matrix pins should be connected to VCC via a pull-up.
 - Keypad diodes should be pointing to the columns, not the rows.
   - "Once the TCA8418 has had the keypad array configured, it will enter idle mode when no keys are being pressed.
      All columns configured as part of the keypad array will be driven low and all rows configured as part of the
      keypad array will be set to inputs, with pull-up resistors enabled. During idle mode, the internal oscillator is turned
      off so that power consumption is low as the device awaits a key press."
  - Missing pull-up on keypad interrupt. Compensated for in software.
  - Fuel gauge interrupt missing pull-up resistor.
  - Mounting holes are not grounded, oops..
  - PG and cahrger lights are on regardless of power state
  - Should have a 3v3 power light
  - Somehow connect vref from the BMP port to the power button on confirmation
  - Should add flow control pins to RF UART "H:4 HCI transport protocol (requires HW flow control from the UART)".
  - Touch screen connector is inverted...
  - LED footprint needs longer pads with some overhang
  - Dispaly connector needs slightly longer pads
  - Display mode pins are backwards...
  - LEDs need a power enable switch tied to the main regulator, they never shut off.