/*******************************************************************************
 * mkl25z128.ld
 *
 * Description:
 *  Linker command file for Kinetis MKL25Z128 targets.
 *
 * Note:
 *  This file defines various global symbols, specifying the start and end of
 *  different sections. This global symbols are used during startup to copy
 *  initialized variables from ROM to SRAM and for initialization of
 *  uninitialized variables.
 *  This file is a modification of the default Kinetis MK21F12 linker file
 *  provided by Freescale. Do not edit this this file unless changes in memory
 *  regions are required.
 *
 * History:
 *  pka, 28/AUG/2014, initial code
 ******************************************************************************/

/* Entry Point. */
ENTRY(Reset_Handler)

/* Memory regions. */
MEMORY
{
    VECTOR_TABLE    (rx) : ORIGIN = 0x00000000, LENGTH = 0xC0
    FLASH_CONFIG    (rx) : ORIGIN = 0x00000400, LENGTH = 0x10
    FLASH           (rx) : ORIGIN = 0x00000800, LENGTH = 128K - 0x800
    SRAM           (rwx) : ORIGIN = 0x1FFFF000, LENGTH = 16K - 0x800
    HEAP           (rwx) : ORIGIN = 0x20002800, LENGTH = 0x400
    STACK          (rwx) : ORIGIN = 0x20002C00, LENGTH = 0x400
}

__StackTop = 0x20003000;    /* end of SRAM */

/* Output sections. */
SECTIONS
{
    /* This section holds the interrupt vector table. */
    .vector_table :
    {
        . = ALIGN(4);
        KEEP(*(.vector_table))
        . = ALIGN(4);
    } > VECTOR_TABLE AT > VECTOR_TABLE

    /* This section holds the flash configuration field. */
    .flash_config :
    {
        . = ALIGN(4);
        KEEP(*(.flash_config))
        . = ALIGN(4);
    } > FLASH_CONFIG AT > FLASH_CONFIG

    /* This section holds executable instructions. */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
    } > FLASH AT > FLASH
    
    /* This section holds read only data. */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH AT > FLASH

    /* This section holds initialized data. */
    .data :
    {
        . = ALIGN(4);
        __data_load = LOADADDR(.data);
        __data_start = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > SRAM AT > FLASH
  
    /* This section holds uninitialized data. */
    .bss :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > SRAM AT > SRAM

    /* This section holds all data which is temporarily stored on the heap. */
    .heap :
    {
        . = ALIGN(4);
        __heap_start = .;
        . = . + LENGTH(HEAP);
        . = ALIGN(4);
        __heap_end = .;
    } > HEAP AT > HEAP
    
    /* This section holds all data which us temporarily stored on the stack. */
    .stack :
    {
        . = ALIGN(4);
        . = . + LENGTH(STACK);
        . = ALIGN(4);
    } > STACK AT > STACK
}
