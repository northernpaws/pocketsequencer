MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH = 2048K /* BANK_1 + BANK_2 */
    RAM      : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
    RAM_D2    : ORIGIN = 0x30000000, LENGTH = 288K
    RAM3_D2   : ORIGIN = 0x30040000, LENGTH = 32K   /* SRAM3, recommended for USB buffers */
    SRAM4_D3   : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
    SDRAM   : ORIGIN = 0xD0000000, LENGTH = 256M   /* external SDRAM */
}

SECTIONS
{
    .sram1_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _ssram1_bss = .;

        PROVIDE(__sram1_bss_start__ = _sram1_bss);
        *(.sram1_bss)
        *(.sram1_bss*)
        . = ALIGN(4);
        _esram1_bss = .;

        PROVIDE(__sram1_bss_end__ = _esram1_bss);
    } > RAM_D2
    
    .ram3_d2 :
    {
        *(.ram3_d2)
    } > RAM3_D2
    
    .sram4_d3 :
    {
        *(.sram5_d3)
    } > SRAM4_D3

    .sdram :
    {
        . = ALIGN(4);
        _ssdram = .;

        PROVIDE(__ssdram_start__ = _sdram);
        *(.sdram)
        *(.sdram*)
        . = ALIGN(4);
        _esdram = .;

        PROVIDE(__sdram_end__ = _esdram);
    } > SDRAM
}
