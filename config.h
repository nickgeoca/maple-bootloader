/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
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

/**
 *  @file config.h
 *
 *  @brief bootloader settings and macro defines
 *
 *
 */

#ifndef __CONFIG_H
#define __CONFIG_H

//******************************************************************************
// LEDs
//******************************************************************************
#define BLINK_FAST      0x50000
#define BLINK_SLOW      0x100000
#define STARTUP_BLINKS  3

//******************************************************************************
// Bootloader
//******************************************************************************
#define BOOTLOADER_WAIT 6
#define FIRST_STAGE_BOOT    // Define this if bootloader is not being loaded.
                            // It will write in the reset ISR and SP to self boot.

//******************************************************************************
// Memory Locations
//******************************************************************************
#define PAGE_SIZE 1024
extern unsigned int __section_table_end;
#define _END_FLASH_ ((unsigned int)(&__section_table_end))
#define USER_CODE_FLASH_OFFSET (((unsigned int)_END_FLASH_ / (unsigned int)PAGE_SIZE) * (unsigned int)PAGE_SIZE + (_END_FLASH_ % PAGE_SIZE? PAGE_SIZE: 0))
#define USER_CODE_RAM_OFFSET   (USER_CODE_FLASH_OFFSET + 0x20000000)

//******************************************************************************
// IDs
//******************************************************************************
#define VEND_ID0        0xAF
#define VEND_ID1        0x1E
#define PROD_ID0        0x03
#define PROD_ID1        0x00

//******************************************************************************
// Board
//******************************************************************************
// Uncomment an MCU model

#define BOARD_SILABS        // SiLabs Cortex M3
// #define BOARD_MAPLE       // STMicro Cortex M3

#define LED             0

#if defined(BOARD_MAPLE)
// LED location
#define LED_BANK        GPIOA
// Button location
#define BUTTON_BANK     GPIOC
#define BUTTON          9
#endif

#endif
