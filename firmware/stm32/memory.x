MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH = 2048K /* BANK_1 + BANK_2 */
    RAM      : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
    RAM3_D2   : ORIGIN = 0x30040000, LENGTH = 32K   /* SRAM3, recommended for USB buffers */
    RAM_D3   : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
    SDRAM   : ORIGIN = 0xD0000000, LENGTH = 256M   /* external SDRAM */
}

SECTIONS
{
    .ram3_d2 :
    {
        *(.ram3_d2)
    } > RAM3_D2
    .ram_d3 :
    {
        *(.ram_d3)
    } > RAM_D3
    .sdram :
    {
        *(.sdram)
    } > SDRAM
}
