/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2012 by Silicon Laboratories.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/
 
MEMORY
{
  /* Define each memory region */

  /* Bootloader gets seven 1024 byte flash pages */
  FLASH (rx) : ORIGIN = 0, LENGTH = 0x3fffc 
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32k */
}
  __top_RAM = 0x20000000 + 0x8000;


SECTIONS
{
	/* For testing. Overwrites stack pointer and Reset ISR*/
	.isr_vector : ALIGN(4)
	{
		PROVIDE(g_pfnVectors = .);
		KEEP(*(.isr_vector));
	} > FLASH
	
	.text : ALIGN(4)
	{
		. = SIZEOF(.isr_vector);

		/* Bootloader code  */
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
		__section_table_end = .;

		KEEP(*(.text))

	} > FLASH
		
	/* Initialized RW data flashed, but referenced in RAM */
	.data : ALIGN(4)
	{
		_data = .;
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
		KEEP( *(.data) )
		KEEP( *(.data*) )
	} > RAM AT>FLASH

	/* bss at RAM */
	.bss : ALIGN(4)
	{
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		_ebss = .;
		PROVIDE(end = .);
		KEEP( *(.bss) )
		KEEP( *(.bss*) )
	} > RAM
	
	PROVIDE(_pvHeapStart = .);
	PROVIDE(_vStackTop = __top_RAM - 0);
	
	DISCARD :
	{
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
		/* Prevents flash overwites and removes bloat */
		*(.comment)
		*(.ARM.attributes)
	}
}
