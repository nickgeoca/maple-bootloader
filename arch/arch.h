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
#if !defined(__ARCH_H)
#define __ARCH_H

//******************************************************************************
// Includes
//******************************************************************************
#include "../config.h"

// ARM Cortex M3
#if defined(BOARD_SILABS) || defined(BOARD_MAPLE)
#include "arm_m3/arm_m3.h"

// ...
#elif defined(______)


// ...
#elif defined(______)
#error Architecture not defined


// Architecture not defined.
#else
#error Architecture not defined
#endif // defined() || ...

//******************************************************************************
// Declarations
//******************************************************************************
void initCPU(void);     // Defined in cpu.c

#endif // !defined(__ARCH_H)
