

# rev 2

 - QUADSPI CS should be NSS capable pin for automatic hardware peripheral control. Use PG6 and swap USB_INT to maybe PD6? 
 - Run LCD display DMA in circular mode, and use half-interrupt and end interrupt to queue double-buffer frame drawing in sync with DMA transfer speeds. 
   - "On the STM32, you can allocate one big array, and use the two halves as double buffer. There is a half-transfer interrupt (when enabled) which tells you that the first half is filled and ready for processing, and the second half is being modified. Then you'll get a transfer complete interrupt when the second half is ready and (in circular mode) it goes on updating the first half again. On higher-end controllers (STM32F4, STM32F7), there are actually two memory addresses for each DMA channel, the buffers halves do not need to be contiguous, and you can even change the address of the inactive buffer on the fly."

 