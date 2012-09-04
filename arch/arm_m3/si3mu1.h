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

#if !defined(__SI3MU1_H_)
#define __SI3MU1_H_

// Includes
#include <stdint.h>

// Defines
#if !defined(ARCH_SI32)
#define ARCH_SI32
#endif


#define CLKCTRL_0  ((uint32_t)0x4002D000)
#define APBCLKG0_SET (CLKCTRL_0 + 0x24)
#define APBCLKG0_CLR (CLKCTRL_0 + 0x28)

#define PBCFG_0         ((uint32_t)0x4002A000)
#define XBAR1_SET       (PBCFG_0 + 0x44)
#define XBAR1_CLR       (PBCFG_0 + 0x48)

// Port bank locations
#define PORT_BANK0 ((uint32_t)0x4002A0A0)
#define PORT_BANK1 ((uint32_t)0x4002A140)
#define PORT_BANK2 ((uint32_t)0x4002A1E0)
#define PORT_BANK3 ((uint32_t)0x4002A320)

// Port bank registers
#define PB_SET          0x04
#define PB_CLR          0x08
#define PB_MSK          0x0C
#define PBMDSEL_SET     0x24
#define PBOUTMD_SET     0x44
#define PBOUTMD_CLR     0x48

// GPIO
#define GPIO_SET(port)          (port + PB_SET)
#define GPIO_CLR(port)          (port + PB_CLR)
#define GPIO_MSK(port)          (port + PB_MSK)
#define GPIO_MDSEL_SET(port)    (port + PBMDSEL_SET)
#define GPIO_OUTMD_SET(port)    (port + PBOUTMD_SET)
#define GPIO_OUTMD_CLR(port)    (port + PBOUTMD_CLR)

#define RSTSRC0_RESETEN ((uint32_t)0x4002D060)


#define WDTIMER_0       ((uint32_t)0x40030000)
#define WDTIMER_WDTKEY  (WDTIMER_0 + 0x30)


#define FLASH_BASE      ((uint32_t)0)
#define RAM_BASE        ((uint32_t)0x20000000)
#define USER_CODE_RAM   (RAM_BASE + USER_CODE_RAM_OFFSET)
#define USER_CODE_FLASH (FLASH_BASE + USER_CODE_FLASH_OFFSET)

//******************************************************************************
// Used for core_m3.h
//******************************************************************************
#define __MPU_PRESENT             0
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    0

typedef enum IRQn
{
  // CPU
  NonMaskableInt_IRQn   = -14, // 2
  MemoryManagement_IRQn = -12, // 4
  BusFault_IRQn         = -11, // 5
  UsageFault_IRQn       = -10, // 6
  SVCall_IRQn           = -5,  // 11
  DebugMonitor_IRQn     = -4,  // 12
  PendSV_IRQn           = -2,  // 14
  SysTick_IRQn          = -1,  // 15
  // MCU
  WDTIMER0_IRQn         = 0,
  PBEXT0_IRQn           = 1,
  PBEXT1_IRQn           = 2,
  RTC0ALRM_IRQn         = 3,
  DMACH0_IRQn           = 4,
  DMACH1_IRQn           = 5,
  DMACH2_IRQn           = 6,
  DMACH3_IRQn           = 7,
  DMACH4_IRQn           = 8,
  DMACH5_IRQn           = 9,
  DMACH6_IRQn           = 10,
  DMACH7_IRQn           = 11,
  DMACH8_IRQn           = 12,
  DMACH9_IRQn           = 13,
  DMACH10_IRQn          = 14,
  DMACH11_IRQn          = 15,
  DMACH12_IRQn          = 16,
  DMACH13_IRQn          = 17,
  DMACH14_IRQn          = 18,
  DMACH15_IRQn          = 19,
  TIMER0L_IRQn          = 20,
  TIMER0H_IRQn          = 21,
  TIMER1L_IRQn          = 22,
  TIMER1H_IRQn          = 23,
  EPCA0_IRQn            = 24,
  PCA0_IRQn             = 25,
  PCA1_IRQn             = 26,
  USART0_IRQn           = 27,
  USART1_IRQn           = 28,
  SPI0_IRQn             = 29,
  SPI1_IRQn             = 30,
  SPI2_IRQn             = 31,
  I2C0_IRQn             = 32,
  I2C1_IRQn             = 33,
  USB0_IRQn             = 34,
  SARADC0_IRQn          = 35,
  SARADC1_IRQn          = 36,
  CMP0_IRQn             = 37,
  CMP1_IRQn             = 38,
  CAPSENSE0_IRQn        = 39,
  I2S0RX_IRQn           = 40,
  I2S0TX_IRQn           = 41,
  AES0_IRQn             = 42,
  VDDLOW_IRQn           = 43,
  RTC0FAIL_IRQn         = 44,
  PMATCH_IRQn           = 45,
  UART0_IRQn            = 46,
  UART1_IRQn            = 47,
  IDAC0_IRQn            = 48,
  IDAC1_IRQn            = 49,
  LPTIMER0_IRQn         = 50,
  PLL0_IRQn             = 51,
  VBUSINVALID_IRQn      = 52,
  VREG0LOW_IRQn         = 53,
} IRQn_Type;

#endif // !defined(__SI3MU1_H)
