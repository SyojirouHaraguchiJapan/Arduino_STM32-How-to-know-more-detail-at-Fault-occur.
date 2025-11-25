# Arduino_STM32 How to know more detail at Fault occur.
## 1. Additional control and data register
Registers

 For distribute Fault processing by cause,  there is special register as below.
- 4.4.9 System handler control and state register (SCB_SHCSR) at p140-141 of PM0056.\
Partial list
  - Bit 18 USGFAULTENA: Usage fault enable bit, set to 1 to enable.
  - Bit 17 BUSFAULTENA: Bus fault enable bit, set to 1 to enable.
  - Bit 16 MEMFAULTENA: Memory management fault enable bit, set to 1 to enable. 

 For check cause and detail, there is special registers as below.
- 4.4.10 Configurable fault status register (SCB_CFSR) at p142-145 of PM0056. \
Partial list
  - Bit 16 UNDEFINSTR: Undefined instruction usage fault.
  - Bit 15 BFARVALID:  Bus Fault Address Register (BFAR) valid.
  - Bit  9 PRECISERR:  Precise data bus error.
  - Bit  8 IBUSERR:    Instruction bus error
  - Bit  7 MMARVALID:  Management Fault Address Register (MMFAR) valid flag.
  - Bit  1 DACCVIOL:   Data access violation flag.
  - Bit  0 IACCVIOL:   Instruction access violation flag
- 4.4.11 Hard fault status register (SCB_HFSR) at p145-146 of RM0056.
- 4.4.12 Memory management fault address register (SCB_MMFAR) at p147 of RM0056.
- 4.4.13 Bus fault address register (SCB_BFAR) at p147 of RM0056.

Notice
- 4.4.14 System control block design hints and tips. at p148 of RM0056.
- 4.4.15 SCB register map. at p148 of RM0056\
For STM32F1 0xE000-ED00, but for STM32L and STM32F2 0xE000-E008. 

## 2. How to check these resisters contents
We store them to BACKUP RAM and check later at next boot-up from reset/reboot.
- Because, in fault processing, we can easy read these registers but hard to output via Serial, USB, I2C and SPI device directly.
- Let us save them into BACKUP RAM in RTC for after reset/reboot.
- BACKUP RAM is built-in for allmost all STM32 device
- And also occur time is important, save with too. 

Description
- Use 10 BACKUP RAM from end according to BKP_NR_DATA_REGS.
- Direct RTC register access.\
Unfortunately, Arduino_STM32 RTC library can't include/call.
- Pack HFSR due to BACKUP RAM shortage.

`C:\Program Files (x86)\Arduino188\hardware\Arduino_STM32\STM32F1\cores\maple\libmaple\util.c`
```
#define USE_BACKUP_RAM
  :
/* This was public as of v0.0.12, so we've got to keep it public. */
/**
 * @brief Fades the error LED on and off
 * @sideeffect Sets output push-pull on ERROR_LED_PIN.
 */
__attribute__((noreturn)) void throb(void) {

#ifdef HAVE_ERROR_LED
    int32  slope   = 1;
    uint32 CC      = 0x0000;
    uint32 TOP_CNT = 0x0200;
    uint32 i       = 0;

  #ifdef USE_COUNTED_BLINK
    uint32 j;
    uint32 cause;
    asm volatile (
        "mov %0, r0"   // assembly instruction: move r0 value to opernd 0 (%0)
        : "=r" (cause) // output operand: C variable cause
        :              // input  operand: none
        : "r0"         // may destroy register
    );
  #endif

  #ifdef USE_BACKUP_RAM
typedef struct rtc_reg_map {
    __IO uint32 CRH;        /**< Control register high */
    __IO uint32 CRL;        /**< Control register high */
    __IO uint32 PRLH;       /**< Prescaler load register high */
    __IO uint32 PRLL;       /**< Prescaler load register low */
    __IO uint32 DIVH;       /**< Prescaler divider register high */
    __IO uint32 DIVL;       /**< Prescaler divider register low */
    __IO uint32 CNTH;       /**< Counter register high */
    __IO uint32 CNTL;       /**< Counter register low */
    __IO uint32 ALRH;       /**< Alarm register high */
    __IO uint32 ALRL;       /**< Alarm register low */
} rtc_reg_map;

#define RTC_BASE            ((struct rtc_reg_map*)0x40002800)
#define RTC_FLAG_RTOFF      (uint32_t) 0x00000020
#define RTC_FLAG_CNF        (uint32_t) 0x00000010
#define RTC_FLAG_RSF        (uint32_t) 0x00000008

    /* get unix time from RTC register by direct access */
    // rtc_clear_sync()
    while (RTC_BASE->CRL & RTC_FLAG_RTOFF == 0);    // wait until previous write complete
    RTC_BASE->CRL &= ~RTC_FLAG_RSF;                 // clear reg sync flag
    // rtc_wait_sync()
    while((RTC_BASE->CRL & RTC_FLAG_RSF) == 0);     // wait until sync
    // rtc_wait_finished()
    while (RTC_BASE->CRL & RTC_FLAG_RTOFF == 0);    // wait until previous write complete
    uint32 h, l;
    do {
        h = RTC_BASE->CNTH & 0xffff;
        l = RTC_BASE->CNTL & 0xffff;
    } while (h != (RTC_BASE->CNTH & 0xffff));

uint16_t bkp_end = BKP_NR_DATA_REGS;

    bkp_enable_writes();

    bkp_write(bkp_end-10, (uint16_t) cause);    // Fault cause number

uint32_t reg_data;
    reg_data = SCB_BASE->CFSR;
    bkp_write(bkp_end-9, (uint16_t) (reg_data >> 16));
    bkp_write(bkp_end-8, (uint16_t) (reg_data & 0xFFFF));

    //reg_data = SCB_BASE->HFSR;
    //bkp_write(bkp_end-8, (uint16_t) (reg_data >> 16));
    //bkp_write(bkp_end-7, (uint16_t) (reg_data & 0xFFFF));
    reg_data = ((SCB_BASE->HFSR >> 16) & 0xC000) | (SCB_BASE->HFSR & 0x0002);
    bkp_write(bkp_end-7, (uint16_t) reg_data);

    reg_data = SCB_BASE->MMFAR;
    bkp_write(bkp_end-6, (uint16_t) (reg_data >> 16));
    bkp_write(bkp_end-5, (uint16_t) (reg_data & 0xFFFF));

    reg_data = SCB_BASE->BFAR;
    bkp_write(bkp_end-4, (uint16_t) (reg_data >> 16));
    bkp_write(bkp_end-3, (uint16_t) (reg_data & 0xFFFF));

    bkp_write(bkp_end-2, (uint16_t) h);
    bkp_write(bkp_end-1, (uint16_t) l);

    bkp_disable_writes();
  #endif

    gpio_set_mode(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_MODE_OUTPUT);

  #ifdef USE_COUNTED_BLINK
    /* Error blink */
    cause &= 0x07;     // limit max 7
    if (cause > 5 || cause < 1) {
        cause = 6;
    }
    while(1) {
        for(j=0; j<cause; j++) {
            gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, 0);   // LED ON
            i=DELAY_COUNT; while (i>0) {int k = sqrt(i); i--; }
            gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, 1);   // LED OFF
            i=DELAY_COUNT; while (i>0) {int k = sqrt(i); i--; }
        }
        for (j=cause; j<7; j++) {
            gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, 1);   // LED OFF
            i=DELAY_COUNT; while (i>0) {int k = sqrt(i); i--; }
            i=DELAY_COUNT; while (i>0) {int k = sqrt(i); i--; }
        }
    }
  #else 
    /* Error fade. */
    while (1) {
        if (CC == TOP_CNT)  {
            slope = -1;
        } else if (CC == 0) {
            slope = 1;
        }

        if (i == TOP_CNT)  {
            CC += slope;
            i = 0;
        }

        if (i < CC) {
            gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, 1);
        } else {
            gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, 0);
        }
        i++;
    }
  #endif
#else
    /* No error LED is defined; do nothing. */
    while (1)
        ;
#endif
}
```
## 3. How to cause each Fault and check saved Fault data in BACKUP RAM
This is "BluePill-Fault-Test.ino" tested on Arduino IDE 1.8.8 + Arduino_STM32.
- Force cause Fault from Arduino side application.
- When Fault occur, CPU save registers to BACKUP RAM at automatic.
- If possible, Arduino side application such as interrupt routine would revive and continue.
- If impossible, Arduino side application such as main routine would stop, only BUILTIN_LED would blink at fade or counted blink mode endless for indicate Fault occur. 
- For recover, it need manual hardware reset or such as automatic WWDG reset.
- At startup from reset/reboot after Fault occur, read BACKUP RAM and output them to Serial line.
- The detection of Fault cause is not sure. So, you must check saved registers data and enusre Fault cause.
```
#define LED_BUILTIN         PC13        // Define onBoard LED port

#define CHK_FAULT_DATA                  // Check previous Fault data at boot up

#define DEBUG_FORCE_FAULT               // Select one of cause from below define
#define BUS_ERROR           1           // Cause BUS error
#define ADR_ERROR           2           // Cause ADR error
#define ZERO_DEVIDE         3           // Cause ZeroDevide error. Coretex-M3 can't but Coretex-M4 can.
#define UNDEFINED_OPCODE    4           // Cause UNDEF error

#include "libmaple/scb.h"               // for SCB register access
#include "libmaple/bkp.h"               // for BKP register access
#include <RTClock.h>                    // RTC clock support library
RTClock rtc(RTCSEL_LSE);
struct tm_t st;

//--------------------
void setup() {

    pinMode(LED_BUILTIN,OUTPUT);        // For onBoard LED drive

    Serial.begin(9600);
    while (!Serial) { delay(100); }
    Serial.println("BluePill-Fault-Test");
    // Enable bits for cause individual fault
    SCB_BASE->SHCSR |= 0x00070000;       // USGA/BUS/MEM FAULT ENAble. See PM0056 4.4.9

#ifdef CHK_FAULT_DATA
    check_fault();
#endif

#ifdef  DEBUG_FORCE_FAULT
    force_fault(BUS_ERROR);
    force_fault(ADR_ERROR);
    force_fault(ZERO_DEVIDE);
    force_fault(UNDEFINED_OPCODE);
#endif
}

//--------------------
void loop() {
    Serial.print(".");
    delay(500);
}

//--------------------
void check_fault() {
uint16_t reg_data, bkp_end; 
uint32_t scb_cfsr, scb_hfsr, scb_mmfar, scb_bfar, unix_time;;
char buf[50];

    bkp_end = BKP_NR_DATA_REGS;
    Serial.print("BKP_NR_DATA_REGS = "); Serial.println(bkp_end,DEC);
    bkp_init();
    reg_data = (uint16_t) bkp_read(bkp_end-10);
    //Serial.print("bkp_read(bkp_end-10) = "); Serial.println(reg_data,DEC);
    //if (reg_data > 0) {
        Serial.println("* Backuped Fault Data");
        Serial.print("* Backuped Cause = "); Serial.println(reg_data,HEX);
        // Backuped SCB_CFSR
        scb_cfsr  = (uint16_t) bkp_read(bkp_end-9) << 16;
        scb_cfsr += (uint16_t) bkp_read(bkp_end-8);
        Serial.print("* Backuped SCB_CFSR = 0x"); Serial.println(scb_cfsr,HEX);
        // Backuped SCB_HFSR
        scb_hfsr = (uint16_t) bkp_read(bkp_end-7);
        scb_hfsr = (scb_hfsr & 0xC000) << 16 + (scb_hfsr & 0x0002); 
        Serial.print("* Backuped SCB_HFSR = 0x"); Serial.println(scb_hfsr,HEX);
        // Backuped SCB_MMFAR
        scb_mmfar  = (uint16_t) bkp_read(bkp_end-6) << 16;
        scb_mmfar += (uint16_t) bkp_read(bkp_end-5);
        Serial.print("* Backuped SCB_MMFAR = 0x"); Serial.println(scb_mmfar,HEX);
        // Backuped SCB_BFAR
        scb_bfar  = (uint16_t) bkp_read(bkp_end-4) << 16;
        scb_bfar += (uint16_t) bkp_read(bkp_end-3);
        Serial.print("* Backuped SCB_BFAR = 0x"); Serial.println(scb_bfar,HEX);
        // Backuped Fault Year/Date/Time
        unix_time  = (uint16_t) bkp_read(bkp_end-2) << 16;
        unix_time += (uint16_t) bkp_read(bkp_end-1);
        rtc.breakTime(unix_time, st);
        sprintf(buf, "* Backuped Fault time = %4d/%02d/%02d %02d:%02d:%02d",
                    st.year+1970, st.month, st.day, st.hour, st.minute, st.second);
        Serial.println(buf);

        bkp_enable_writes();
        for(int i=10; i>0; i--) {
            bkp_write(bkp_end-i, (uint16_t) 0);    // Clear Fault Cause data
        }
        bkp_disable_writes();
        //reg_data = (uint16_t) bkp_read(bkp_end-10);
        //Serial.print("* Fault Cause (cleared) = "); Serial.println((reg_data),DEC);
    //}
}

//--------------------
void force_fault(int fault_code) {
uint32_t adr, offset, data;

  switch(fault_code) {
    case BUS_ERROR:
      // Bus Error check
      Serial.println("** Bus Error check >>");
      for (uint8_t i=0; i<16; i++) {
          delay(100);
          adr = i * 0x10000000;
          Serial.print("Address = 0x"); Serial.print(adr,HEX);
          data = *(__IO uint32_t *) adr;
          Serial.print(" : Data = 0x"); Serial.println(data,HEX);
      }
      break;
    case ADR_ERROR:
      // Address Error check
      Serial.println("** Address Error check >>");
      for (uint8_t i=0; i<16; i++) {
          delay(100);
          adr = i * 0x10000000 + 1;
          Serial.print("Address = 0x"); Serial.print(adr,HEX);
          data = *(__IO uint32_t *) adr;
          Serial.print(" : Data = 0x"); Serial.println(data,HEX);
      }
      break;
    case ZERO_DEVIDE:
      Serial.println("** Zero Devide Error check >>");
      Serial.println("*** STM32F103 can't execute 'Zero Devide'.");
      //asm volatile ("sdiv r0,r1,r2"); // only Coretex-M4, Coretex-M3 do not have this instruction
      //asm volatile ("udiv r0,r1,r2"); // only Coretex-M4, Coretex-M3 do not have this instruction
      break;
    case UNDEFINED_OPCODE:
      Serial.println("** Unefined Opecode Error check >>");
      asm volatile (".byte 0xFF, 0xFF, 0xFF, 0xFF");
      break;
    default:
      break;
  }
}
```
