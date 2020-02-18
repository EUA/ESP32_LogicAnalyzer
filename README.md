# ESP32 LogicAnalyzer
A *SUMP* compatible 16Bit Logic Analyzer for ESP32 MCUs.

![PulseView](/ESP32_LogicAnalyzer_in_PulseView.png)

* Use Arduino to compile and flash your ESP32.

* Default port is **UART0** and default baudrate is **912600**. Please set "Core Debug Level"=None when using UART0 as OLS port.

* You can use **UART2** for high speed OLS communication with using`#define USE_SERIAL2_FOR_OLS @ ESP32_LogicAnalyzer.h.` You can also Core Debug Levels on this mode. Default baudrate is **3M**.

* **GPIO23** used for I2S input clk. Don't use it for IO or change it to an unused pin from code.

* Code use **DMA** and could capture speeds up to **20 Mhz**, which is limit for ESP32 I2S port.

* **Maximum 128k samples**. Even using 8 bit capturing mode. WROOVER modules might allow more. Not tested due lack of HW. the [esp32.com](https://esp32.com/viewtopic.php?t=14135&p=55462) just sends me a WROOVER development board :+1:. Will look how to use its external RAM.

* Analog input is **NOT** available.

* RLE compression is under development. 8bit RLE encoder that supports 20Mhz is almost ready.

* Using Serial (UART0) require you flush the RX bytes due ESP32 prints boot post debug messages to Serial. You can use this patch https://sigrok.org/bugzilla/show_bug.cgi?id=1025 for PulseView/Sigrok to work **or** please pull **GPIO15** to ground ( and also set **"Core Debug Level"=None** to let UART0 only used for OLS.)

If you like it, why not to say thanks or support via [Patreon](https://www.patreon.com/EUA)



This project steals some code from [esp32-cam-demo](https://github.com/igrr/esp32-cam-demo) for I2S DMA and [Arduino Logic Analyzer](https://github.com/gillham/logic_analyzer) as SUMP protocol "template".
