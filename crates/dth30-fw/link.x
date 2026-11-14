/* The image is loaded at SDRAM physical 0x00100000 and runs in kseg0.
   .head must come first: the host reads the vector table at +0x110/+0x120. */
ENTRY(_reset)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x80100000, LENGTH = 0x00200000
}

SECTIONS
{
    .head : { KEEP(*(.head)) } > RAM
    .text : { *(.text .text.*) } > RAM
    .rodata : ALIGN(16) { *(.rodata .rodata.*) *(.lit4 .lit8) } > RAM
    .data : ALIGN(16)
    {
        *(.data .data.*)
        _gp = ALIGN(16) + 0x7FF0;
        *(.got .got.*)
        *(.sdata .sdata.*)
    } > RAM
    /* Real bytes at the end, so that the raw image is a multiple of 16 bytes. */
    .tail : ALIGN(16) { QUAD(0) QUAD(0) } > RAM

    .bss (NOLOAD) : ALIGN(16)
    {
        __bss_start = .;
        *(.sbss .sbss.*) *(.bss .bss.*) *(COMMON)
        . = ALIGN(16);
        __bss_end = .;
    } > RAM
    .stack (NOLOAD) : ALIGN(16)
    {
        . += 0x4000;
        __stack_top = .;
    } > RAM

    /DISCARD/ : { *(.MIPS.abiflags) *(.reginfo) *(.pdr) *(.eh_frame*) *(.comment) *(.note*) }
}
