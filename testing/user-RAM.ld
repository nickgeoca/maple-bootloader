

 MEMORY
{
 
  /* Define each memory region */
  FLASH (rx) : ORIGIN = 0, LENGTH = 0x3ffff
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32k */
}
  __top_RAM = 0x20000000 + 0x8000;
  _start_user_app = 0x1400;

SECTIONS
{
	
	.main_start : ALIGN(4)
	{
		. = _start_user_app;
		LONG(__top_RAM);
		*(.main_start)
		KEEP(*(.main_start));
	} > RAM
	
	.text : ALIGN(4)
	{
			
		PROVIDE(__main_start = .);
		*(.text)
		*(.text*)
		/* Read only variables (constants) */
		*(.rodata)
		*(.rodata.*)
		
		/* 
		Global Section Table 
		References locations of non constant globals to
		copy to RAM. 
		*/
		__section_table_start = .;
		__data_section_table = .;
		LONG(LOADADDR(.data));
		LONG(    ADDR(.data)) ;
		LONG(  SIZEOF(.data));
		__data_section_table_end = .;
		__bss_section_table = .;
		LONG(    ADDR(.bss));
		LONG(  SIZEOF(.bss));
		__bss_section_table_end = .;
		__section_table_end = . ;
		
		KEEP(*(.text))
	} > RAM
	
	/* Initialized RW data flashed, but referenced in RAM */
	.data : ALIGN(4)
	{
		_data = .;
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
		KEEP( *(.data) )
		KEEP( *(.data*) )
	} > RAM

	/* bss at RAM */
	.bss : ALIGN(4)
	{
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		_ebss = .;
		KEEP( *(.bss) )
		KEEP( *(.bss*) )
	} > RAM
	
	DISCARD :
	{
		*(.data)
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
		/* Prevents flash overwites and removes bloat */
		*(.comment)
		*(.ARM.attributes)
	} > RAM
}
