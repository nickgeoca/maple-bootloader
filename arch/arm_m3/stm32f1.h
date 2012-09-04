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

#ifndef __HARDWARE_H
#define __HARDWARE_H

// Defines
#define ARCH_STM32

/* macro'd register and peripheral definitions */
#define RCC   ((u32)0x40021000)
#define FLASH ((u32)0x40022000)
#define GPIOA ((u32)0x40010800)
#define GPIOC ((u32)0x40011000)

#define RCC_CR      RCC
#define RCC_CFGR    (RCC + 0x04)
#define RCC_CIR     (RCC + 0x08)
#define RCC_AHBENR  (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x1C)

#define GPIO_CRL(port)  port
#define GPIO_CRH(port)  (port+0x04)
#define GPIO_IDR(port)  (port+0x08)
#define GPIO_ODR(port)  (port+0x0c)
#define GPIO_BSRR(port) (port+0x10)

#define SCS_BASE   ((u32)0xE000E000)
#define NVIC_BASE  (SCS_BASE + 0x0100)
#define SCB_BASE   (SCS_BASE + 0x0D00)

#define SCB_VTOR (SCB+0x08)
#define STK_CTRL (STK+0x00)

/* AIRCR  */
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | (u32)0x04);

/* temporary copyage of example from kiel */
#define __VAL(__TIMCLK, __PERIOD) ((__TIMCLK/1000000UL)*__PERIOD)
#define __PSC(__TIMCLK, __PERIOD)  (((__VAL(__TIMCLK, __PERIOD)+49999UL)/50000UL) - 1)
#define __ARR(__TIMCLK, __PERIOD) ((__VAL(__TIMCLK, __PERIOD)/(__PSC(__TIMCLK, __PERIOD)+1)) - 1)



/* todo: there must be some major misunderstanding in how we access
   regs. The direct access approach (GET_REG) causes the usb init to
   fail upon trying to activate RCC_APB1 |= 0x00800000. However, using
   the struct approach from ST, it works fine...temporarily switching
   to that approach */
typedef struct {
    vu32 CR;
    vu32 CFGR;
    vu32 CIR;
    vu32 APB2RSTR;
    vu32 APB1RSTR;
    vu32 AHBENR;
    vu32 APB2ENR;
    vu32 APB1ENR;
    vu32 BDCR;
    vu32 CSR;
} RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC)

typedef struct {
    vu32 ISER[2];
    u32  RESERVED0[30];
    vu32 ICER[2];
    u32  RSERVED1[30];
    vu32 ISPR[2];
    u32  RESERVED2[30];
    vu32 ICPR[2];
    u32  RESERVED3[30];
    vu32 IABR[2];
    u32  RESERVED4[62];
    vu32 IPR[15];
} NVIC_TypeDef;

typedef struct {
    u8 NVIC_IRQChannel;
    u8 NVIC_IRQChannelPreemptionPriority;
    u8 NVIC_IRQChannelSubPriority;
    bool NVIC_IRQChannelCmd; /* TRUE for enable */
} NVIC_InitTypeDef;

typedef struct {
    vuc32 CPUID;
    vu32 ICSR;
    vu32 VTOR;
    vu32 AIRCR;
    vu32 SCR;
    vu32 CCR;
    vu32 SHPR[3];
    vu32 SHCSR;
    vu32 CFSR;
    vu32 HFSR;
    vu32 DFSR;
    vu32 MMFAR;
    vu32 BFAR;
    vu32 AFSR;
} SCB_TypeDef;

#endif
