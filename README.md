# Pocket Sequencer

[Board Schematic](https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2Fnorthernpaws%2Fpocketsequencer%2Fblob%2Fmain%2Fhardware%2Fboards%2Fv0.1%2Fpocket_synth.kicad_pro)
[Board v0.1 Errata](hardware/boards/v0.1/README.md)

This repository contains my project for an STM32H7-based sequencer, synthesizer, and sampler, all in a small phone-sized pocketable form factor. 

  * 480 MHz Arm® Cortex®-M7 32-bit Processor (`STM32H743ZIT6`)
    * Floating-point unit
    * DSP instructions
    * High-speed memory controller
    * Quad SPI interface for flash
    * High-speed USB host and device mode
    * Several SAI audio blocks
  * 2.8" 240x320 TFT IPS Display (`ER-TFT028A2-4`)
  * 24 bit Audio Codec (`NAU88C22YG`)
    * DAC: 94dB SNR
    * ADC: 90dB SNR
    * 1W 8Ω BTL speaker driver
    * 40mW 16Ω headphone driver
  * Optional Bluetooth and Wifi (`nRF5430`)
    * Wirelessly connect Bluethooth headphones and speakers
    * Supports various high-fedality audio protocols
    * BLE MIDI
    * Wifi sample management and transfer
  * Interal stereo 8W speakers speaker
  * Internal MEMs microphone
  * SDRAM for sample pool and audio processing
    * Part allocated to in-memory sample pool for project
    * Part allocated to audio effects processing
    * Variable sizes in similar footprint available, up to 512Mbit:
        * 16bit * 48000Hz = 768,000b/s = 0.768Mb/s
        * 128Mb / 0.768Mb/s = 166.66 seconds of mono audio (`IS42S16800J-7TL`)
        * 256Mb / 0.768Mb/s = 333.33 seconds of mono audio (`IS42S16160J-7TLI`)
        * 512Mb / 0.768Mb/s = 666.66 seconds of mono audio (`IS42S16320F-7TLI`)
  * Micro SD Card
    * Saving and loading projects
      * Transfer and back up projects to a computer
    * Saving and loading samples
      * Load samples from a computer
      * Save samples recorded on the device
  * 3.3v NOR Flash (`MT25QUxxxxxx1xW7` 256M/128M/64M)
    * Stores device settings
    * Small storage space for projects and samples without SD card
    * Provides storage for cross-SD card samples
  * USB-C
    * USB-PD fast charging
    * Possible future audio interface mode
    * USB MIDI in and out
  * MIDI in/out via TRS minijack type A
  * 16 Multi-Purpose primary keys
    * Hot-swappable Kalih V1 Choc keyswitches
    * Addressable RGB LEDs for state indicators
  * Accelerometer and Gyroscope (`LSM6DS3`)
    * Can be used as sequencer parameters
    * Can be recorded for automation inputs

## Step Sequencer

The step sequencer on the device is modeled after popular semantics used by hardware sequencers, such as the Elektron Digitone and Digitakt, OP-1, OP-Z, Pocket Operators, Woovebox, etc.

> The hard requirements of the step sequencer and audio engine are not defined yet because they are under active development.

  * As many projects as the SD card or internal memory can store
    * 128 Patterns (TBD)
      * 8-16 Audio or MIDI tracks per pattern (TBD)
    * 8-24 Songs (arrangements of patterns) (TBD)
  * 8-16 voice polyphony (TBD)
    * 2-3 LFO per voice (TBD)

## Audio Samples

The primary audio engine runs in 32-bit and supports stereo audio.

### Sampling

The sampling engine runs 16bit 44.1Khz and saves and loads samples in single-channel mono.

Samples recoded from the internal or external mic are sampled at 24 bits into the device's SDRAM to provide a decent noise floor, and then filtered and downsampled to 16 bits for saving after they've been edited by the user.

The 16bit @ 44.1Khz sampling parameters provdes a good balance between audio quality, and reasonable utilization of the SDRAM without needing extremely large and expensive SDRAM chips:

```
16bit * 48000Hz = 768,000b/s = 0.768Mb/s
128Mb / 0.768Mb/s = 166.66 seconds of mono audio
256Mb / 0.768Mb/s = 333.33 seconds of mono audio
512Mb / 0.768Mb/s = 666.66 seconds of mono audio
```

> Note that some of the space calculated above (actual amount will be configurable) is allocated to the audio effects processing chain, i.e. for reverb and delay. 

## References

### Debugging

 - HardFault: Under XPERIPHERALS in Cortex-debug check the `CFSR_UFSR_BFSR_MMFSR` register to see the fault code and reference: https://community.st.com/t5/stm32-mcus/how-to-debug-a-hardfault-on-an-arm-cortex-m-stm32/ta-p/672235#toc-hId--1471762501
