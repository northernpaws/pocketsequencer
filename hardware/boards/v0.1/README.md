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