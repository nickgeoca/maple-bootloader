/*****************************************************************************
Title:     STK500v2 compatible bootloader
           Modified for Wiring board ATMega128-16MHz
Author:    Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:      $Id: stk500boot.c,v 1.11 2006/06/25 12:39:17 peter Exp $
Compiler:  avr-gcc 3.4.5 or 4.1 / avr-libc 1.4.3
Hardware:  All AVRs with bootloader support, tested with ATmega8
License:   GNU General Public License

Modified:  Worapoht Kornkaewwattanakul <dev@avride.com>   http://www.avride.com
Date:      17 October 2007
Update:    1st, 29 Dec 2007 : Enable CMD_SPI_MULTI but ignore unused command by return 0x00 byte response..
Compiler:  WINAVR20060421
Description: add timeout feature like previous Wiring bootloader

DESCRIPTION:
    This program allows an AVR with bootloader capabilities to
    read/write its own Flash/EEprom. To enter Programming mode
    an input pin is checked. If this pin is pulled low, programming mode
    is entered. If not, normal execution is done from $0000
    "reset" vector in Application area.
    Size fits into a 1024 word bootloader section
    when compiled with avr-gcc 4.1
    (direct replace on Wiring Board without fuse setting changed)

USAGE:
    - Set AVR MCU type and clock-frequency (F_CPU) in the Makefile.
    - Set baud rate below (AVRISP only works with 115200 bps)
    - compile/link the bootloader with the supplied Makefile
    - program the "Boot Flash section size" (BOOTSZ fuses),
      for boot-size 1024 words:  program BOOTSZ01
    - enable the BOOT Reset Vector (program BOOTRST)
    - Upload the hex file to the AVR using any ISP programmer
    - Program Boot Lock Mode 3 (program BootLock 11 and BootLock 12 lock bits) // (leave them)
    - Reset your AVR while keeping PROG_PIN pulled low // (for enter bootloader by switch)
    - Start AVRISP Programmer (AVRStudio/Tools/Program AVR)
    - AVRISP will detect the bootloader
    - Program your application FLASH file and optional EEPROM file using AVRISP

Note:
    Erasing the device without flashing, through AVRISP GUI button "Erase Device"
    is not implemented, due to AVRStudio limitations.
    Flash is always erased before programming.

    AVRdude:
    Please uncomment #define REMOVE_CMD_SPI_MULTI when using AVRdude.
    Comment #define REMOVE_PROGRAM_LOCK_BIT_SUPPORT to reduce code size
    Read Fuse Bits and Read/Write Lock Bits is not supported

NOTES:
    Based on Atmel Application Note AVR109 - Self-programming
    Based on Atmel Application Note AVR068 - STK500v2 Protocol

LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*****************************************************************************/

//************************************************************************
//* Edit History
//************************************************************************
//* Jul  7, 2010    <MLS> = Mark Sproul msproul@skycharoit.com
//* Jul  7, 2010    <MLS> Working on mega2560. No Auto-restart
//* Jul  7, 2010    <MLS> Switched to 8K bytes (4K words) so that we have room for the monitor
//* Jul  8, 2010    <MLS> Found older version of source that had auto restart, put that code back in
//* Jul  8, 2010    <MLS> Adding monitor code
//* Jul 11, 2010    <MLS> Added blinking LED while waiting for download to start
//* Jul 11, 2010    <MLS> Added EEPROM test
//* Jul 29, 2010    <MLS> Added recchar_timeout for timing out on bootloading
//* Aug 23, 2010    <MLS> Added support for atmega2561
//* Aug 26, 2010    <MLS> Removed support for BOOT_BY_SWITCH
//* Sep  8, 2010    <MLS> Added support for atmega16
//************************************************************************



#include    <inttypes.h>
#include    <stdlib.h>
#include    "command.h"
#include    "common.h"
/*
 * Uncomment the following lines to save code space
 */

#define SPM_PAGESIZE 1024


/*
 * define CPU frequency in Mhz here if not defined in Makefile
 */
#define F_CPU 20000000UL

/*
 * UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */

#ifndef BAUDRATE
    #define BAUDRATE 115200
#endif


/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           2
#define CONFIG_PARAM_SW_MINOR           0x0A

/*
 * Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
 * (adjust BOOTSIZE below and BOOTLOADER_ADDRESS in Makefile if you want to change the size of the bootloader)
 */

#define BOOTSIZE    (USER_CODE_FLASH_OFFSET)
#define APP_BEGIN   (BOOTSIZE)

/*
 * Signature bytes are not available in avr-gcc io_xxx.h
 */
#define SIGNATURE_BYTES 0x1e9801
// FIXME


/*
 * Macro to calculate UBBR from XTAL and baudrate
 */


/*
 * States used in the receive state machine
 */
#define ST_START        0
#define ST_GET_SEQ_NUM  1
#define ST_MSG_SIZE_1   2
#define ST_MSG_SIZE_2   3
#define ST_GET_TOKEN    4
#define ST_GET_DATA     5
#define ST_GET_CHECK    6
#define ST_PROCESS      7


typedef uint32_t address_t;


/*
 * function prototypes
 */
static void sendchar(char c);
static unsigned char recchar(void);

//*****************************************************************************
// erase page in flash
typedef struct flash_reg_map {
   volatile uint32_t  CFGR;
   volatile uint32_t  CFGR_SET;
   volatile uint32_t  CFGR_CLR;
   uint32_t       reserved0[37];
   volatile uint32_t  WRADDRR;
   uint32_t       reserved1[3];
   volatile uint32_t  WRDATAR;
   uint32_t       reserved2[3];
   volatile uint32_t  KEYR;
   uint32_t       reserved3[3];
   volatile uint32_t  TCR;
} flash_reg_map;
#define FLASH_BASE ((struct flash_reg_map*)0x4002E000)
static inline void _flash_unlock_multi(void)
{
    // Enable vdd suply monitor and set as reset source
    (*(volatile uint32_t*)((uint32_t)&(*(volatile uint32_t*)0x4002F000) + (4 << !(1)))) = (0x80000000);
    (*(volatile uint32_t*)((uint32_t)&(*(volatile uint32_t*)0x4002D060) + (4 << !(1)))) = (0x00000004);


    FLASH_BASE->KEYR = 165;
    FLASH_BASE->KEYR = 242;
}

static inline void _flash_lock_multi(void)
{
    FLASH_BASE->KEYR = 90;
}
void boot_page_erase(uint32_t eraseAddress)
{
    // Enable flash erase
    (*(volatile uint32_t*)((uint32_t)&(FLASH_BASE->CFGR) + (4 << !(1)))) = ((1 << 18));

    // Unlock flash for multiple write
    _flash_unlock_multi();

    // Set address register
    FLASH_BASE->WRADDRR = eraseAddress;

    // Erase page
    FLASH_BASE->WRDATAR = 0;

    // Lock
    _flash_lock_multi();

    // Disable erase mode
    (*(volatile uint32_t*)((uint32_t)&(FLASH_BASE->CFGR) + (4 << !(0)))) = ((1 << 18));
}

void flash_write_data(uint32_t address, uint16_t data[], int32_t count)
{
    uint32_t up_count;

    // Unlock flash for multiple write
    _flash_unlock_multi();


    // Set address register
    FLASH_BASE->WRADDRR = address;

    for (up_count = 0; up_count != count; up_count++) {
        FLASH_BASE->WRDATAR = data[up_count];
    }

    _flash_lock_multi();
}
void init_flashctrl(void)
{
    // set flash control clock
    (*(volatile uint32_t*)((uint32_t)0x4002D020 + (4 << !(1)))) = (1 << 30);
}

//*****************************************************************************
#define SIM3_DELAY_US_MULT (7 * F_CPU / 1000000 / 20)

static void delay_us(uint32_t us) {
    us *= SIM3_DELAY_US_MULT;

    /* fudge for function call overhead  */
    us--;

    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

void delay_ms(unsigned int timedelay)
{
    unsigned int i;
    for (i=0;i<timedelay;i++)
    {
        delay_us(1000);
    }
}




/////////////
typedef struct usart_reg_map
{
    volatile uint32_t  CONFIG; // Base Address + 0x0
    volatile uint32_t  CONFIG_SET;
    volatile uint32_t  CONFIG_CLR;
    uint32_t       reserved0;
    volatile uint32_t  MODE; // Base Address + 0x10
    volatile uint32_t  MODE_SET;
    volatile uint32_t  MODE_CLR;
    uint32_t       reserved1;
    volatile uint32_t  FLOWCN; // Base Address + 0x20
    volatile uint32_t  FLOWCN_SET;
    volatile uint32_t  FLOWCN_CLR;
    uint32_t       reserved2;
    volatile uint32_t  CONTROL; // Base Address + 0x30
    volatile uint32_t  CONTROL_SET;
    volatile uint32_t  CONTROL_CLR;
    uint32_t       reserved3;
    volatile uint32_t  IPDELAY; // Base Address + 0x40
    uint32_t       reserved4[3];
    volatile uint32_t  B_RATE; // Base Address + 0x50
    uint32_t       reserved5[3];
    volatile uint32_t  FIFOCN; // Base Address + 0x60
    volatile uint32_t  FIFOCN_SET;
    volatile uint32_t  FIFOCN_CLR;
    uint32_t       reserved6;
    volatile uint32_t  DATA; // Base Address + 0x70
    uint32_t       reserved7[3];
} usart_reg_map;

#define USART0_BASE ((usart_reg_map*)0x40000000)
#define TRACE_CHAR(c)   do { \
                            *((volatile uint8_t *)&USART0_BASE->DATA) = (uint8_t)c; \
                            *((volatile uint8_t *)&USART0_BASE->DATA) = (uint8_t)'\r'; \
                            *((volatile uint8_t *)&USART0_BASE->DATA) = (uint8_t)'\n'; \
                            while (0x00070000 & USART0_BASE->FIFOCN); \
                        } while (0);
#define TRACE_SNGL_CHAR(c)   do { \
                            *((volatile uint8_t *)&USART0_BASE->DATA) = (uint8_t)c; \
                            while (0x00070000 & USART0_BASE->FIFOCN); \
                        } while (0);
void INIT_TRACE(void)
{
    uint32_t baud;

    // setup crossbar
    (*(volatile uint32_t*)((uint32_t)0x4002A020 + (4 << !(1)))) = (1 << 0);

    // Gpio rx (already configured as input)
    //Gpio tx
    (*(volatile uint32_t*)((uint32_t)0x4002A0E0 + (4 << !(1)))) = (1);

    // set usart clock
    (*(volatile uint32_t*)((uint32_t)0x4002D020 + (4 << !(1)))) = (1 << 2);

    // tx configuration
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->CONFIG) + (4 << !(0)))) = (0x07000000 | 0x00020000 | 0x00180000 | 0x40000000);
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->CONFIG) + (4 << !(1)))) = ((3 << 24) | (1 << 16) | (1 << 18) | (1 << 19));
    // rx configuration
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->CONFIG) + (4 << !(0)))) = (0x00000700 | 0x00000018 | 0x00004000);
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->CONFIG) + (4 << !(1)))) = ((3 << 8) | (1 << 0) | (1 << 2) | (1 << 3));

    // Full duplex
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->MODE) + (4 << !(0)))) = (0x08000000);

    // Set baud rate. Should use apb frequency, but the apb frequency will always be cpu freqency in bootloader.
    baud = F_CPU / (2 * BAUDRATE) - 1;
    USART0_BASE->B_RATE = baud | (baud << 16);

    // Enable USART
    (*(volatile uint32_t*)((uint32_t)&(USART0_BASE->CONTROL) + (4 << !(1)))) = ((1 << 15) | (1U << 31));
}
#define UART1_BASE ((usart_reg_map*)0x40003000)
static void sendchar(char c);
void init_uart1(void)
{
    uint32_t baud;
    // Skip gpio pins up to tx/rx on portbank 2 and 3
    (*(volatile uint32_t*)((uint32_t)0x4002A210 + (4 << !(1)))) = 0x0000FFFF;
    (*(volatile uint32_t*)((uint32_t)0x4002A350 + (4 << !(1)))) = 0x00007FFF;
    (*(volatile uint32_t*)((uint32_t)0x4002A350 + (4 << !(0)))) = (1 << 4) | (1 << 5);

    // setup crossbar
    (*(volatile uint32_t*)((uint32_t)0x4002A040 + (4 << !(1)))) = (1 << 16);

    // Gpio rx (already configured as input)
    //Gpio tx
    (*(volatile uint32_t*)((uint32_t)0x4002A360 + (4 << !(1)))) = (1 << 4);

    // set usart clock
    (*(volatile uint32_t*)((uint32_t)0x4002D020 + (4 << !(1)))) = (1 << 5);

    // tx configuration
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->CONFIG) + (4 << !(0)))) = (0x07000000 | 0x00020000 | 0x00180000 | 0x40000000);
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->CONFIG) + (4 << !(1)))) = ((3 << 24) | (1 << 16) | (1 << 18) | (1 << 19));
    // rx configuration
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->CONFIG) + (4 << !(0)))) = (0x00000700 | 0x00000018 | 0x00004000);
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->CONFIG) + (4 << !(1)))) = ((3 << 8) | (1 << 0) | (1 << 2) | (1 << 3));

    // Full duplex
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->MODE) + (4 << !(0)))) = (0x08000000);

    // Set baud rate. Should use apb frequency, but the apb frequency will always be cpu freqency in bootloader.
    baud = F_CPU / (2 * BAUDRATE) - 1;
    UART1_BASE->B_RATE = baud | (baud << 16);

    // Enable USART
    (*(volatile uint32_t*)((uint32_t)&(UART1_BASE->CONTROL) + (4 << !(1)))) = ((1 << 15) | (1U << 31));
}

//*****************************************************************************
/*
 * send single byte to USART, wait until transmission is completed
 */
static void sendchar(char c)
{
    // send char
    *((volatile uint8_t *)&UART1_BASE->DATA) = (uint8_t)c;

    // Wait till tx fifo is empty
    while (0x00070000 & UART1_BASE->FIFOCN);

}
/////////////

//************************************************************************
static int  Serial_Available(void)
{
    return 0x7 & UART1_BASE->FIFOCN;
}


//*****************************************************************************
/*
 * Read single byte from USART, block if no data available
 */
static unsigned char recchar(void)
{
    //block until char available;
    while (!(0x7 & UART1_BASE->FIFOCN)) ;
    return *(volatile uint8_t*)(&UART1_BASE->DATA);
}

#define MAX_TIME_COUNT  (F_CPU >> 1)
//*****************************************************************************
static unsigned char recchar_timeout(void)
{
uint32_t count = 0;
    while (!(0x7 & UART1_BASE->FIFOCN))
    {
        // wait for data
        count++;
        if (count > MAX_TIME_COUNT)
        {
        unsigned int    data;
            if (checkUserCode(USER_CODE_FLASH)) {
                    jumpToUser(USER_CODE_FLASH);
            }
            count   =   0;
        }
    }
    return *(volatile uint8_t*)(&UART1_BASE->DATA);
}



//*****************************************************************************
void stk500v2(void)
{
    address_t       address         =   0;
    address_t       eraseAddress    =   0;
    unsigned char   msgParseState;
    unsigned int    i              =   0;
    unsigned char   checksum        =   0;
    unsigned char   seqNum          =   0;
    unsigned int    msgLength       =   0;
    unsigned char   msgBuffer[1200] __attribute__ ((aligned (2)));
    unsigned char   c, *p;
    unsigned char   isLeave = 0;

    unsigned long   boot_timeout;
    unsigned long   boot_timer;
    unsigned int    boot_state;


    boot_timer  =   0;
    boot_state  =   0;
    boot_timeout    =   20000000; // 7 seconds , approx 2us per step when optimize "s"

    /*
     * Branch to bootloader or application code ?
     */

    // init uart0
    init_uart1();;
    init_flashctrl();
    INIT_TRACE();

    TRACE_CHAR('\n');
    TRACE_CHAR('\r');
    TRACE_CHAR('\n');
    TRACE_CHAR('\r');
    while (boot_state==0)
    {
        while ((!(Serial_Available())) && (boot_state == 0))        // wait for data
        {
            delay_us(1);
            boot_timer++;
            if (boot_timer > boot_timeout)
            {
                boot_state  =   1; // (after ++ -> boot_state=2 bootloader timeout, jump to main 0x00000 )
            }
            if ((boot_timer % 20000) == 0)
            {
                //* toggle the LED
                *(volatile uint32_t*)(0x4002A320 + 12) =  ((1 << 0) << 16) | (~(*(volatile uint32_t *)0x4002A320) & 0xFFFF);
            }
        }
        boot_state++; // ( if boot_state=1 bootloader received byte from UART, enter bootloader mode)
    }


    if (boot_state==1)
    {
        for(;;)
        {
            /*
             * Collect received bytes to a complete message
             */
            msgParseState   =   ST_START;
            while ( msgParseState != ST_PROCESS )
            {
                if (boot_state==1)
                {
                    boot_state  =   0;
                    c           =   *(volatile uint8_t*)(&UART1_BASE->DATA);
                }
                else
                {
                    c   =   recchar();
                }



                switch (msgParseState)
                {
                    case ST_START:
                        if( c == MESSAGE_START )
                        {
                            msgParseState = ST_GET_SEQ_NUM;
                            checksum = MESSAGE_START^0;
                        }
                        break;

                    case ST_GET_SEQ_NUM:
                        if ( (c == 1) || (c == seqNum) )
                        {
                            seqNum = c;
                            msgParseState = ST_MSG_SIZE_1;
                            checksum ^= c;
                        }
                        else
                        {
                            msgParseState = ST_START;
                        }
                        break;

                    case ST_MSG_SIZE_1:
                        msgLength = c<<8;
                        msgParseState = ST_MSG_SIZE_2;
                        checksum ^= c;
                        break;

                    case ST_MSG_SIZE_2:
                        msgLength |= c;
                        msgParseState = ST_GET_TOKEN;
                        checksum ^= c;
                        break;

                    case ST_GET_TOKEN:
                        if ( c == TOKEN )
                        {
                            msgParseState = ST_GET_DATA;
                            checksum ^= c;
                            i = 0;
                        }
                        else
                        {
                            msgParseState = ST_START;
                        }
                        break;

                    case ST_GET_DATA:
                        msgBuffer[i++] = c;
                        checksum ^= c;
                        if ( i == msgLength )
                        {
                            msgParseState = ST_GET_CHECK;
                        }
                        break;

                    case ST_GET_CHECK:
                        if( c == checksum )
                        {
                            msgParseState = ST_PROCESS;
                        }
                        else
                        {
                            msgParseState = ST_START;
                        }
                        break;
                }//switch
            }//while(msgParseState)

            /*
             * Now process the STK500 commands, see Atmel Appnote AVR068
             */

            switch (msgBuffer[0])
            {
                case CMD_SPI_MULTI:
                TRACE_CHAR('a');
                {
                    unsigned char answerByte;
                    unsigned char flag=0;
                    if ( msgBuffer[4]== 0x30 )
                    {
                        unsigned char signatureIndex = msgBuffer[6];

                        if ( signatureIndex == 0 )
                            answerByte = (SIGNATURE_BYTES >>16) & 0x000000FF;
                        else if ( signatureIndex == 1 )
                            answerByte = (SIGNATURE_BYTES >> 8) & 0x000000FF;
                        else
                            answerByte = SIGNATURE_BYTES & 0x000000FF;
                    }
                    else if ( msgBuffer[4] & 0x50 )
                    {
                        if (msgBuffer[4] == 0x50)
                        {
                            answerByte  =   0;
                        }
                        else if (msgBuffer[4] == 0x58)
                        {
                            answerByte  =   0x80;
                        }
                    }
                    else
                    {
                        answerByte = 0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
                    }
                    if ( !flag )
                    {
                        msgLength = 7;
                        msgBuffer[1] = STATUS_CMD_OK;
                        msgBuffer[2] = 0;
                        msgBuffer[3] = msgBuffer[4];
                        msgBuffer[4] = 0;
                        msgBuffer[5] = answerByte;
                        msgBuffer[6] = STATUS_CMD_OK;
                    }
                }

                break;

                case CMD_SIGN_ON:
                    TRACE_CHAR('0');
                    msgLength       =   11;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   8;
                    msgBuffer[3]    =   'A';
                    msgBuffer[4]    =   'V';
                    msgBuffer[5]    =   'R';
                    msgBuffer[6]    =   'I';
                    msgBuffer[7]    =   'S';
                    msgBuffer[8]    =   'P';
                    msgBuffer[9]    =   '_';
                    msgBuffer[10]   =   '2';

                    break;

                case CMD_GET_PARAMETER:
                    TRACE_CHAR('1');
                    {
                        unsigned char value;

                        switch(msgBuffer[1])
                        {
                        case PARAM_BUILD_NUMBER_LOW:
                            value = CONFIG_PARAM_BUILD_NUMBER_LOW;
                            break;
                        case PARAM_BUILD_NUMBER_HIGH:
                            value = CONFIG_PARAM_BUILD_NUMBER_HIGH;
                            break;
                        case PARAM_HW_VER:
                            value = CONFIG_PARAM_HW_VER;
                            break;
                        case PARAM_SW_MAJOR:
                            value = CONFIG_PARAM_SW_MAJOR;
                            break;
                        case PARAM_SW_MINOR:
                            value = CONFIG_PARAM_SW_MINOR;
                            break;
                        default:
                            value = 0;
                            break;
                        }
                        msgLength = 3;
                        msgBuffer[1] = STATUS_CMD_OK;
                        msgBuffer[2] = value;

                    }
                    break;

                case CMD_SET_PARAMETER:
                    TRACE_CHAR('?');
                case CMD_ENTER_PROGMODE_ISP:
                    TRACE_CHAR('?');
                case CMD_LEAVE_PROGMODE_ISP:
                    TRACE_CHAR('2');
                    msgLength = 2;
                    msgBuffer[1] = STATUS_CMD_OK;
                    break;

                case CMD_READ_SIGNATURE_ISP:
                    TRACE_CHAR('3');
                    {
                        unsigned char signatureIndex    =   msgBuffer[4];
                        unsigned char signature;

                        if ( signatureIndex == 0 )
                            signature   =   (SIGNATURE_BYTES >>16) & 0x000000FF;
                        else if ( signatureIndex == 1 )
                            signature   =   (SIGNATURE_BYTES >> 8) & 0x000000FF;
                        else
                            signature   =   SIGNATURE_BYTES & 0x000000FF;

                        msgLength       =   4;
                        msgBuffer[1]    =   STATUS_CMD_OK;
                        msgBuffer[2]    =   signature;
                        msgBuffer[3]    =   STATUS_CMD_OK;
                    }

                    break;

                case CMD_READ_LOCK_ISP:
                    TRACE_CHAR('4');
                    msgLength       =   4;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   0;//boot_lock_fuse_bits_get( GET_LOCK_BITS );
                    msgBuffer[3]    =   STATUS_CMD_OK;

                    break;

                case CMD_READ_FUSE_ISP:
                    TRACE_CHAR('5');
                    {
                        unsigned char fuseBits;

                        if ( msgBuffer[2] == 0x50 )
                        {
                            if ( msgBuffer[3] == 0x08 )
                                fuseBits    =   0;//boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
                            else
                                fuseBits    =   0;//boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
                        }
                        else
                        {
                            fuseBits    =   0;
                        }
                        msgLength       =   4;
                        msgBuffer[1]    =   STATUS_CMD_OK;
                        msgBuffer[2]    =   fuseBits;
                        msgBuffer[3]    =   STATUS_CMD_OK;
                    }

                    break;

                case CMD_CHIP_ERASE_ISP:
                    TRACE_CHAR('6');
                    eraseAddress    =   0;
                    msgLength       =   2;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    break;

                case CMD_LOAD_ADDRESS:

                {
                    uint32_t ld_adr_tmp = 1;
                    TRACE_SNGL_CHAR('7');
                    TRACE_SNGL_CHAR(':');
                    TRACE_SNGL_CHAR(' ');
                    TRACE_SNGL_CHAR('0');
                    TRACE_SNGL_CHAR('x');
                       while (ld_adr_tmp < 5) {
                           TRACE_SNGL_CHAR((msgBuffer[ld_adr_tmp] >> 4) + (msgBuffer[ld_adr_tmp] >> 4 > 9 ? 0x31-10: 0) + 0x30);
                           TRACE_SNGL_CHAR((msgBuffer[ld_adr_tmp] & 0xf) + (msgBuffer[ld_adr_tmp] & 0xf > 9 ? 0x31-10: 0) + 0x30);

                          ld_adr_tmp += 1;
                       }
                       TRACE_CHAR(' ');
                    address =   ( ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;
                    address = 0x1c00;
                    msgLength       =   2;
                    msgBuffer[1]    =   STATUS_CMD_OK;

                }
                    break;

                case CMD_PROGRAM_FLASH_ISP:
                    TRACE_CHAR('?');
                case CMD_PROGRAM_EEPROM_ISP:
                    TRACE_CHAR('8');
                    {
                        unsigned int    size    =   ((msgBuffer[1])<<8) | msgBuffer[2];
                        unsigned char   *p  =   msgBuffer+10;
                        uint16_t    data;
                        unsigned char   highByte, lowByte;
                        address_t       tempaddress =   address;


                        if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
                        {
                            // erase only main section (bootloader protection)
                            if (eraseAddress >= APP_BEGIN)
                            {
                                boot_page_erase(eraseAddress);  // Perform page erase
                                eraseAddress += SPM_PAGESIZE;   // point to next page to be erase
                            }

                            flash_write_data(address, (uint16_t*)msgBuffer, size / 2);
                            address += size;
                        }
                        msgLength   =   2;
                        msgBuffer[1]    =   STATUS_CMD_OK;
                    }
                    break;

                case CMD_READ_FLASH_ISP:
                    TRACE_CHAR('?');
                case CMD_READ_EEPROM_ISP:
                    TRACE_CHAR('9');
                    {
                        unsigned int    size    =   ((msgBuffer[1])<<8) | msgBuffer[2];
                        unsigned char   *p      =   msgBuffer+1;
                        msgLength               =   size+3;

                        *p++    =   STATUS_CMD_OK;
                        if (msgBuffer[0] == CMD_READ_FLASH_ISP )
                        {
                            unsigned int data;

                            // Read FLASH
                            do {
                                data    =   *(volatile uint16_t *)address;
                                *p++    =   (unsigned char)data;        //LSB
                                *p++    =   (unsigned char)(data >> 8); //MSB
                                address +=  2;                          // Select next word in memory
                                size    -=  2;
                            }while (size);
                        }
                        else
                        {
                            /* Read EEPROM */
                            do {
                                *p++    =   0;               // Send EEPROM data
                                size--;
                            } while (size);
                        }
                        *p++    =   STATUS_CMD_OK;
                    }
                    break;

                default:
                    TRACE_CHAR('-');
                    TRACE_CHAR((msgBuffer[0] >> 4) + (msgBuffer[0] >> 4 > 9 ? 0x31-10: 0) + 0x30);
                    TRACE_CHAR((msgBuffer[0] & 0xf) + (msgBuffer[0] & 0xf > 9 ? 0x31-10: 0) + 0x30);

                    TRACE_CHAR('-');
                    msgLength       =   2;
                    msgBuffer[1]    =   STATUS_CMD_FAILED;

                    break;
            }

            /*
             * Now send answer message back
             */
            sendchar(MESSAGE_START);
            checksum = MESSAGE_START^0;

            sendchar(seqNum);
            checksum ^= seqNum;

            c = ((msgLength>>8)&0xFF);
            sendchar(c);
            checksum ^= c;

            c = msgLength&0x00FF;
            sendchar(c);
            checksum ^= c;

            sendchar(TOKEN);
            checksum ^= TOKEN;

            p = msgBuffer;
            while ( msgLength )
            {
                c = *p++;
                sendchar(c);
                checksum ^=c;
                msgLength--;
            }
            sendchar(checksum);
            seqNum++;
        }
    }


    return;
}








































