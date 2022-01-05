OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(vectors)

/* Highest address of the user mode stack */
_sstack = 0x20008000;
_estack = 0x20080000;
_physical_end_itcm = 0x7FFFC;


MEMORY 
{
    ITCM0 : ORIGIN = 0x00000000, LENGTH = 0x80000
    DTCM0 : ORIGIN = 0x20000000, LENGTH = 0x80000
}

SECTIONS
{
    .isr_vectors :
    {
        _vectors_start = . ;
        *(.isr_vectors)
        _vectors_end = . ;
    } > ITCM0

    .start :
    {
        *(.start.*)
    } > ITCM0

    _stext = . ;
    .text :
    {
        *(.text*)
        . = ALIGN(4) ;
    } > ITCM0
    _etext = . ;

    .itcm_ext :
    {
        *(.h264*)
        . = ALIGN(4);
    } > ITCM0

    _data_start = .;
    
    .rodata :
    {
        *(.rodata*)
        . = ALIGN(4) ;
    }  >DTCM0 AT> ITCM0

    .data : 
    {
        *(.data*)
        . = ALIGN(4) ;
        *(.ramcode)
    } >DTCM0 AT> ITCM0

    .data_e :
    {
        _data_end = . ;
    } AT> ITCM0

    .itcm_e :
    {
        . = ALIGN(4);
        end_itcm = .;
    } > ITCM0
		
    .bss :
    {
        _bss_start = . ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > DTCM0

    end = . ;
}


