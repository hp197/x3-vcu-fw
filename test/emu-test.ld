/* On-target unit-test images (pio test -e emu-test): x3emu only, never flashed.

   Same layout as x3-vcu-fw.ld (vectors at the slot-0 base 0x08001000, 32 KB SRAM,
   Artery startup_at32f415.s symbols) with two differences:

   - FLASH is 120 KB, 0x08001000-0x0801EFFF: the product code plus Unity, the test
     support and the suite do not fit the 60 KB slot. It stops below the identity pages
     at 0x0801F000, which x3emu provides. It covers slot 1
     (0x08010000-0x0801EFFF), so a test must not let product code erase or program slot 1
     for real (fake the flash calls, test/tst_bsp_fakes.h).
   - The .data/.bss of the product objects (the ones built from src/) are grouped between
     tst_product_{data,bss}_{start,end}, so tst_reset_product_state() can put every
     product global and static back to its boot value without touching Unity's or the
     test support's own state. */
_estack = 0x20008000;      /* top of 32 KB SRAM */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08001000, LENGTH = 120K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(Reset_Handler)

SECTIONS
{
    /* Vector table first in slot 0: the bootloader reads the initial SP and Reset_Handler at
       the image start, and main sets VTOR to it. KEEP because no code references it. VTOR needs
       the table aligned to its vector count rounded up to a power of two: 92 -> 128 vectors,
       512 bytes. */
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    ASSERT(ADDR(.isr_vector) == ORIGIN(FLASH), "vector table must be first in slot 0")
    ASSERT(ADDR(.isr_vector) % 512 == 0, "VTOR needs 512-byte alignment (92 vectors)")

    .text : {
        *(.text*) *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4); _sdata = .;
        tst_product_data_start = .;
        */src/*.o(.data .data.*)
        . = ALIGN(4);
        tst_product_data_end = .;
        *(.data*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH

    .bss : {
        . = ALIGN(4); _sbss = .; __bss_start__ = .;
        tst_product_bss_start = .;
        */src/*.o(.bss .bss.* COMMON)
        . = ALIGN(4);
        tst_product_bss_end = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .; __bss_end__ = .;
    } > RAM

    /* .init/.fini (crti/crtn _init/_fini): nothing calls them, __libc_init_array is empty (runtime.c). */
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) *(.init) *(.fini) }
}
