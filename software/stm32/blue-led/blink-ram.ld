ENTRY(blink)

/* Use AXI SRAM above the USB bootloader's workspace. */
MEMORY {
    RAM (rwx) : ORIGIN = 0x24010000, LENGTH = 64K
}

SECTIONS {
    .image : {
        LONG(ORIGIN(RAM) + LENGTH(RAM))
        LONG(blink | 1)
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.data .data.*)
        /* Include zero-initialized variables in the downloaded RAM image. */
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
    } > RAM
    ASSERT(SIZEOF(.image) + 4096 <= LENGTH(RAM), "Program leaves too little stack space")
    /DISCARD/ : { *(.ARM.exidx* .ARM.extab*) }
}
