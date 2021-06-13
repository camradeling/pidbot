MEMORY
{
    intvec_start : ORIGIN = 0x00000000, LENGTH = 0xEC
    rom : ORIGIN = 0x080000EC, LENGTH = 0x1ff14
    ram : ORIGIN = 0x20000000, LENGTH = 0x5000
}

SECTIONS
{
    .intvec : { *(.intvec*) } > intvec_start
    .text : { *(.text*) } > rom
    .bss  : { *(.bss*) } > ram
}