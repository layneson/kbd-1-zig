MEMORY
{
    FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 32K
    RAM (rxw)     : ORIGIN = 0x20000000, LENGTH = 8K
}

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

STACK_SIZE = 256;

ENTRY(Reset_Handler)

PROVIDE(__stack_top = ORIGIN(RAM) + LENGTH(RAM));

SECTIONS {
    .text : {
        LONG(__stack_top);
        KEEP(*(.isr_vector))
        KEEP(*(.irq_vector))
        *(.text)
        . = ALIGN(4);
    } > FLASH

    .rodata : {
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    __exidx_start = .;
    .ARM.exidx : {
        *(.ARM.exidx*)
        . = ALIGN(4);
    } > FLASH
    __exidx_end = .;

    __text_end = .;

    .data : AT(__text_end) {
        __data_start = .;
        *(SORT_BY_ALIGNMENT(.data*))
        . = ALIGN(4);
    } > RAM
    __data_size = SIZEOF(.data);

    .bss (NOLOAD) : {
        __bss_start = .;
        *(SORT_BY_ALIGNMENT(.bss*) SORT_BY_ALIGNMENT(COMMON*))
        . = ALIGN(4);
    } > RAM
    __bss_size = SIZEOF(.bss);

    /* DWARF 4 */
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_aranges 0 : { *(.debug_aranges) }
    .debug_frame 0 : { *(.debug_frame) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_loc 0 : { *(.debug_loc) }
    .debug_macinfo 0 : { *(.debug_macinfo) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_pubtypes 0 : { *(.debug_pubtypes) }
    .debug_ranges 0 : { *(.debug_ranges) }
    .debug_str 0 : { *(.debug_str) }
    .debug_types 0 : { *(.debug_types) }
}
