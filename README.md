# Arduino_STM32 How to know more detail at Fault occur.
## 1. Additional control and data register
 For distribute Fault processing by cause,  there is special registers as below.
- 4.4.9 System handler control and state register (SCB_SHCSR) at p140-141 of PM0056.
- 4.4.10 Configurable fault status register (SCB_CFSR) at p142-145 of PM0056.
- 4.4.11 Hard fault ststus register (SCB_HFSR) at p145-146 of RM0056.
- 4.4.12 Memory management fault address register (SCB_MMFAR) at p147 of RM0056.
- 4.4.13 Bus fault address register (SCB_BFAR) at p147 of RM0056.
And notice
- 4.4.14 System control block design hints and tips. at p148 of RM0056.
- 4.4.15 SCB register map. at p148 of RM0056\
For STM32F1 0xE000-ED00, but for STM32L and STM32F2 0xE000-E008. 
## 2. How to check these resisters contents
We store them to BACKUP RAM and check after next reset/reboot.
- Because, in fault processing, we can easy read these registers but hard to output via Serial, USB, I2C and SPI device directly.
- let us save them into BACKUP RAM in RTC for after reset/reboot.
- BACKUP RAM is built-in for allmost all STM32 device
- And also occur time is important, save with too. 
