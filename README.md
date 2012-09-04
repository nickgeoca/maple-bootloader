maple-bootloader
================

Bootloader firmware for a Silabs ARM Cortex-M3 development board from Silicon Labs. This code was borrowed from the Leaflabs' maple-bootloader project. The main differences between the two is the removal of the USB files and DFU functionality in this version. The makefile and linker script was rewritten.

This code was designed with porting in mind. Although the STM32 based Maple board has not been tested, it should require minimal coding to function.

The repo files and bootloader testing are explained below.

FILES -------------------------------------------------------------------------

testing/*
  - test application for bootloader. Includes a document for testing.

arch/*
  - Attempt at porting the STM32 and SiM3 architectures. Similar to U-boots arch folder, they ARM M3 chips are in the same folder.
  
*.c and *.h
  - Source code. main.c is used to initalize non-constant global variables. This is unusual and in the future the asm files should be used to initialize the data.
  
TESTING -----------------------------------------------------------------------

testing/test procedure.txt
  - Use this as a guide to test the bootloader.
