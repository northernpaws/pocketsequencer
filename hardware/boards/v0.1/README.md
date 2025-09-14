# v0.1

## Observations

### Routing

  - In next revision, use larger STM32 package. Ran out of pins on the left side due to the FMC and SDRAM, and had to route several GPIO across the entire board, including some interrupt traces in the power layer. Ran out of ADC pins for the velocity pads and had to add an $8 IO expander.
  - Smaller Wifi module would be ideal.