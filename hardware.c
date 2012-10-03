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
 *  @file hardware.c
 *
 *  @brief init routines to setup clocks, interrupts, also destructor functions.
 *  does not include USB stuff. EEPROM read/write functions.
 *
 */

#include "hardware.h"

void setPin(u32 bank, u8 pin) {
    u32 pinMask = 0x1 << (pin);
#if defined(ARCH_SI32)
    SET_REG(GPIO_SET(bank), pinMask);
#elif defined(ARCH_STM32)
    SET_REG(GPIO_BSRR(bank), pinMask);
#endif // defined(ARCH_SI32)
}

void resetPin(u32 bank, u8 pin) {

    u32 pinMask = 0x1 << (pin);
#if defined(ARCH_SI32)
    SET_REG(GPIO_CLR(bank), pinMask);
#elif defined(ARCH_STM32)
    SET_REG(GPIO_BSRR(bank), pinMask << 16);
#endif // defined(ARCH_SI32)
}

void strobePin(u32 bank, u8 pin, u8 count, u32 rate) {
    resetPin(bank, pin);

    u32 c;
    while (count-- > 0) {
        for (c = rate; c > 0; c--) {
            asm volatile("nop");
        }
        setPin(bank, pin);
        for (c = rate; c > 0; c--) {
            asm volatile("nop");
        }
        resetPin(bank, pin);
    }
}

void setupISRs(void)
{
    // Disable all interrupts
    u32 i;

    for (i = 1; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
        // Disable interrupt
        NVIC->ICER[i] = 0xFFFFFFFF;
        // Clear pending interrupt
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    //SET_REG(STK_CTRL, 0x04); /* disable the systick, which operates separately from nvic */
}

void systemReset(void) {
#if defined(ARCH_SI32)
    SET_REG(XBAR1_CLR, 0x80000000);

    // Set all ports as digital inputs
    SET_REG(GPIO_OUTMD_CLR(PORT_BANK0), 0x0000FFFF);
    SET_REG(GPIO_SET(PORT_BANK0), 0x0000FFFF);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK0), 0x0000FFFF);

    SET_REG(GPIO_OUTMD_CLR(PORT_BANK1), 0x0000FFFF);
    SET_REG(GPIO_SET(PORT_BANK1), 0x0000FFFF);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK1), 0x0000FFFF);

    SET_REG(GPIO_OUTMD_CLR(PORT_BANK2), 0x0000FFFF);
    SET_REG(GPIO_SET(PORT_BANK2), 0x0000FFFF);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK2), 0x0000FFFF);

    SET_REG(GPIO_OUTMD_CLR(PORT_BANK3), 0x00007FFF);
    SET_REG(GPIO_SET(PORT_BANK3), 0x00007FFF);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK3), 0x00007FFF);

#elif defined(ARCH_STM32)
    SET_REG(RCC_CR, GET_REG(RCC_CR)     | 0x00000001);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xF8FF0000);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFEF6FFFF);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFFFBFFFF);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xFF80FFFF);

    SET_REG(RCC_CIR, 0x00000000);  /* disable all RCC interrupts */
#endif // defined(ARCH_STM32)
}

void setupCLK(void)
{
#if defined(ARCH_SI32)
    // Enable APB clock to Port Bank Modules
    SET_REG(APBCLKG0_SET, 0x00000002);

#elif defined(ARC_STM32)
    /* enable HSE */
    SET_REG(RCC_CR, GET_REG(RCC_CR) | 0x00010001);
    while ((GET_REG(RCC_CR) & 0x00020000) == 0); /* for it to come on */

    /* enable flash prefetch buffer */
    SET_REG(FLASH_ACR, 0x00000012);

    /* Configure PLL */
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) | 0x001D0400); /* pll=72Mhz,APB1=36Mhz,AHB=72Mhz */
    SET_REG(RCC_CR, GET_REG(RCC_CR)     | 0x01000000); /* enable the pll */
    while ((GET_REG(RCC_CR) & 0x03000000) == 0);         /* wait for it to come on */

    /* Set SYSCLK as PLL */
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) | 0x00000002);
    while ((GET_REG(RCC_CFGR) & 0x00000008) == 0); /* wait for it to come on */
#endif // defined(ARCH_SI32)
}

void setupLED(void) {
#if defined(ARCH_SI32)
    // Enable Crossbar 1 to use LED
    SET_REG(XBAR1_SET, 0x80000000);

    // Set LED 3 as push pull output
    SET_REG(GPIO_OUTMD_SET(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK2), 0x00000400);

    // GPIO Port Bank setup
    // Enable LED 3
    SET_REG(GPIO_OUTMD_CLR(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_SET(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK2), 0x00000400);

#elif defined(ARC_STM32)
    // todo, swap out hardcoded pin/bank with macro
    u32 rwmVal; /* read-write-modify place holder var */

    /* Setup APB2 (GPIOA) */
    rwmVal =  GET_REG(RCC_APB2ENR);
    rwmVal |= 0x00000004;
    SET_REG(RCC_APB2ENR, rwmVal);

    /* Setup GPIOA Pin 5 as PP Out */
    SET_REG(GPIO_CRL(GPIOA), 0x00100000);

    rwmVal =  GET_REG(GPIO_CRL(GPIOA));
    rwmVal &= 0xFF0FFFFF;
    rwmVal |= 0x00100000;
    SET_REG(GPIO_CRL(GPIOA), rwmVal);

    setPin(GPIOA, 5);
#endif // defined(ARCH_SI32)
}

bool checkUserCode(u32 usrAddr) {
    u32 sp = *(vu32*)usrAddr;

    if ((sp & 0x2FFE0000) == 0x20000000) {
        return (TRUE);
    } else {
        return (FALSE);
    }
}

void jumpToUser(u32 usrAddr) {
    typedef void (*funcPtr)(void);
    // The start of the bootloader references the user application location
    u32 appStart = usrAddr;
    funcPtr userMain;

    systemReset(); // resets clocks and periphs, not core regs

    // Offset by 4 where Wiring app starts. Add 1 because it will be a thumb instruction.
    userMain = (funcPtr)(*(vu32 *)(appStart + 4));

    // set the users stack ptr
    __set_MSP(*(vu32 *)appStart);

    // Go!
    userMain();
}

void systemHardReset(void) {
#if defined(ARCH_SI32)
    SET_REG(RSTSRC0_RESETEN, 0x00000040);
#elif defined(ARCH_STM32)
    SCB_TypeDef *rSCB = (SCB_TypeDef *) SCB_BASE;

    /* Reset  */
    rSCB->AIRCR = (u32)AIRCR_RESET_REQ;
#endif
    /*  should never get here */
    while (1) {
        asm volatile("nop");
    }
}
