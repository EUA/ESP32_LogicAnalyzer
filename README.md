3000# ESP32 LogicAnalyzer
A *SUMP* compatible 16Bit Logic Analyzer for ESP32 MCUs.

![PulseView](/ESP32_LogicAnalyzer_in_PulseView.png)

* Use Arduino to compile and flash your ESP32.
* Uses **ESP32 I2S DMA** and could capture speeds up to **20 Mhz**.
* Support **8 bit** and **16 bit** operations.
* Maximum **128k** samples. (Even using 8bit capturing mode.)
* RLE compression supported.
* Analog input is **NOT** available.
* ~~WROOVER modules support **2M** samples but only up to **2 Mhz** due bandwith limit on PSRAM access.~~ Under development.
* Default OLS port is **UART0** and default baudrate is **912600**.
* You can use **UART2** for high speed OLS communication by using **USE_SERIAL2_FOR_OLS** macro at ESP32_LogicAnalyzer.h file. Default OLS baudrate is **3M** on this mode.
* **WARNING:** 
  - For OLS port at UART0
    - Please pull **GPIO15** to ground (this will silence boot up messages.)
    - Set **"Core Debug Level"=None** before compiling code at arduino.
  - Set **"Core Debug Level"=None** for > 10Mhz capture operations.
  - **GPIO23** used for I2S input clk. Don't use it for IO or change it to an unused pin from code.




If you like it, why not to say thanks or support via [Patreon](https://www.patreon.com/EUA)



This project steals some code from [esp32-cam-demo](https://github.com/igrr/esp32-cam-demo) for I2S DMA and [Arduino Logic Analyzer](https://github.com/gillham/logic_analyzer) as SUMP protocol "template".
