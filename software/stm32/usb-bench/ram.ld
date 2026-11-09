ENTRY(start)
MEMORY { RAM (rwx) : ORIGIN = 0x24010000, LENGTH = 128K }
stack_top = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
    .vectors : { KEEP(*(.vectors)) } > RAM
    .text : { *(.text .text.*) *(.rodata .rodata.*) } > RAM
    .data : { *(.data .data.*) } > RAM
    .bss (NOLOAD) : {
        . = ALIGN(8); bss_start = .; *(.bss .bss.* COMMON)
        . = ALIGN(8); bss_end = .; end = .;
    } > RAM
    ASSERT(end + 16384 < stack_top, "Insufficient RAM for stack and library heap")
    /DISCARD/ : { *(.ARM.exidx* .ARM.extab* .comment) }
}
