#include <stdint.h>
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)

// Defines
#define SET_REG(addr,val) do { *(vu32*)(addr)=val; } while(0)
#define GET_REG(addr)     (*(vu32*)(addr))


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


#define USER_CODE_RAM     ((u32)0x20000C00)
#define USER_CODE_FLASH   ((u32)0x08005000)

// These two blocks of declarations are used initialize global data.
static void setupGlobals(void);
static void data_init(unsigned int romstart, unsigned int start, unsigned int len);
static void bss_init(unsigned int start, unsigned int len);

extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;
u32 tmp = 0x08000;


//
// Copy the data sections from flash to SRAM.
//
void setupGlobals(void)
{

    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }
}
void setPin(u32 bank, u8 pin) {
    u32 pinMask = 0x1 << (pin);
    SET_REG(GPIO_SET(bank), pinMask);
}

void resetPin(u32 bank, u8 pin) {

    u32 pinMask = 0x1 << (pin);
    SET_REG(GPIO_CLR(bank), pinMask);
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

__attribute__ ((section(".main_start")))
int
main(void)
{
    /*
    // Enable APB clock to Port Bank Modules
    SET_REG(APBCLKG0_SET, 0x00000002);

    // Enable Crossbar 1, where the example drives LEDs, samples buttons, and
    // outputs the RTC clock.
    SET_REG(XBAR1_SET, 0x80000000);

    // Set LED 3 as push pull output
    SET_REG(GPIO_OUTMD_SET(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK2), 0x00000400);

    // GPIO Port Bank setup
    // Enable LED 3
    SET_REG(GPIO_OUTMD_CLR(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_SET(PORT_BANK2), 0x00000400);
    SET_REG(GPIO_MDSEL_SET(PORT_BANK2), 0x00000400);
     */

    // Turn off watchdog
    SET_REG(WDTIMER_WDTKEY, 165);
    SET_REG(WDTIMER_WDTKEY, 221);

    setupGlobals();

    while (1) {
        strobePin(PORT_BANK2, 10, 6, tmp);
        tmp = tmp ^ 0x28000;
    }

}



void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

