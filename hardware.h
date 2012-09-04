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

#include "common.h"

// Defines
#define SET_REG(addr,val) do { *(vu32*)(addr)=val; } while(0)
#define GET_REG(addr)     (*(vu32*)(addr))

// Includes
void setPin(u32 bank, u8 pin);
void resetPin(u32 bank, u8 pin);
bool readPin(u32 bank, u8 pin);
void strobePin(u32 bank, u8 pin, u8 count, u32 rate);
void setupISRs(void);
void systemHardReset(void);
void systemReset(void);
void setupCLK(void);
void setupLED(void);
void setupBUTTON(void);
bool checkUserCode(u32 usrAddr);
void jumpToUser(u32 usrAddr);


#endif
