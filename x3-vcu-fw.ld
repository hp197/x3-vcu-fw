/* AT32F415CBT7 VCU application: slot 0 of flash @ 0x08001000 (60 KB), 32 KB SRAM.
   The x3 bootloader launches this image; its vector table sits at 0x08001000.
   Compatible with Artery's startup_at32f415.s (expects _estack, _sidata,
   _sdata/_edata, _sbss/_ebss). */
_estack = 0x20008000;      /* top of 32 KB SRAM */

MEMORY
{
    /* Slot 0, 0x08001000-0x0800FFFF. Slot 1 above it holds the OTA staging copy. */
    FLASH (rx)  : ORIGIN = 0x08001000, LENGTH = 60K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Reset_Handler is the gc-sections root; without it the linker never pulls the
   startup object (vector table + Reset_Handler) out of libcmsis.a. */
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

    /* Code between the vector table and the slot-0 header at 0x08001400: whole objects that
       fit the 656-byte gap. If they grow too large the .app_header address below overlaps
       and the link fails. */
    .text_low : {
        *scheduler.o(.text*)
        *buzzer.o(.text*)
        *runtime.o(.text*)
    } > FLASH

    /* Slot-0 header (config.h APP_HEADER_SECTION): banner +0x00 (16 B), OTA auth key +0x20
       (16 B), build id +0x30 (6 B). */
    .app_header 0x08001400 : {
        KEEP(*(.app_header.banner))
        . = 0x20;
        KEEP(*(.app_header.key))
        . = 0x30;
        KEEP(*(.app_header.build_id))
        . = ALIGN(4);
    } > FLASH
    ASSERT(ADDR(.app_header) == 0x08001400, "slot-0 header must sit at 0x08001400")

    .text : {
        *(.text*) *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4); _sdata = .;
        *(.data*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH

    .bss : {
        . = ALIGN(4); _sbss = .; __bss_start__ = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .; __bss_end__ = .;
    } > RAM

    /* .init/.fini (crti/crtn _init/_fini): nothing calls them, __libc_init_array is empty (runtime.c). */
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) *(.init) *(.fini) }
}
