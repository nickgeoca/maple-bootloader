maple-bootloader
================

Bootloader firmware for a Silabs ARM Cortex-M3 development board (SiM3U167 MCU) from Silicon Labs. This code was borrowed from the Leaflabs' maple-bootloader project. The main differences between the two is the removal of the USB files and DFU functionality in this version. The makefile and linker script was rewritten.

An attempt was made to keep the STM32 and SiM3 MCUs seperated, but the STM32 code has not been tested. The STM32 code will likely need some more work.

The repo files, bootloader testing, and Precision32 IDE project setup are explained below. 

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

Precision32 IDE Project Setup -------------------------------------------------

maple-bootloader
  1. Use Git to clone the repository to a folder.
  2. In Precision32 create a new project.
    * Select "Makefile Project with Existing Code". 
	* Select the existing bootloader folder. Under Languages: Unselect C++. Undner Toolchain: Select Code Red MCU Tools. Click finish.
  3. In the IDE, right click on the project and click properties. 
    * Click C/C++ build in the left pane.
	* Under C/C++ build, uncheck "Use default build command" and type "make -f Makefile" under "Build command"
	* Go to C/C++ Build->MCU settings and click SiM3U167.
	* Save the changes and exit the properties window by clicking OK.
  4. In the IDE, right click on the "testing" folder and select properties.
    * Click C/C++ Build and check "Exclude resource from build".
	* Click OK to save the changes and exit.
	
test-application
  1. Repeat steps 1 through 3, but reference the testing folder as the source location when creating the project.