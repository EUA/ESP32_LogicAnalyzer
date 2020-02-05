Might WROOVER version modules allows more. Not tested.

Analog input or RLE compression is NOT available, at least right now.
# ESP32 LogicAnalyzer
A *SUMP* compatible 16Bit Logic Analyzer for ESP32 MCUs.

![PulseView](/ESP32_LogicAnalyzer_in_PulseView.png)

* Use Arduino to compile and flash your ESP32.

* **GPIO23** used for I2S input clk. Don't use it for IO or change it to an unused pin from code.

* Default baud rate is **912600**. You can use UART2 for high speed OLS communication but you need high speed external RS232 TTL device.

* Code use **DMA** and could capture speeds up to **20 Mhz**, which is limit for ESP32 I2S port 

* **Maximum 128k samples**. Even using 8 bit capturing mode. Might WROOVER version modules allows more. Not tested.

* Analog input or RLE compression is **NOT** available, at least right now.

* Using Serial(0) require you flush the RX bytes since ESP32 prints boot debug messages to Serial. You can use this patch https://sigrok.org/bugzilla/show_bug.cgi?id=1025 for PulseView/Sigrok work proper.


If you like it, why not to say thanks or support via [Patreon](https://www.patreon.com/EUA)



This project steals some code from [esp32-cam-demo](https://github.com/igrr/esp32-cam-demo) for I2S DMA and [Arduino Logic Analyzer](https://github.com/gillham/logic_analyzer) as SUMP protocol "template".
