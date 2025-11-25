#define LED_BUILTIN         PC13        // Define onBoard LED port

#define CHK_FAULT_DATA                  // Check previous Fault data at boot up

#define DEBUG_FORCE_FAULT               // Select one of cause from below define
#define BUS_ERROR           1           // Cause BUS error
#define ADR_ERROR           2           // Cause ADR error
#define ZERO_DEVIDE         3           // Cause ZeroDevide error. Coretex-M3 can't but Coretex-M4 can.
#define UNDEFINED_OPCODE    4           // Cause UNDEF error

#include "libmaple/scb.h"               // for SCB register access
#include "libmaple/bkp.h"               // for BKP register access

//void check_fault();
//void force_fault();
    
#include <RTClock.h>                    // RTC clock support library
RTClock rtc(RTCSEL_LSE);
struct tm_t st;

//--------------------
void setup() {

    pinMode(LED_BUILTIN,OUTPUT);        // For onBoard LED drive

    Serial.begin(9600);
    while (!Serial) { delay(100); }
    Serial.println("BluePill-Fault-Test2-251009b");
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
