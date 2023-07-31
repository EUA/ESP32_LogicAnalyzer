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


## Quick start guide
1. Use PlatformIO (in VS Code) to open the project folder
2. Modify to use UART0 (MicroUSB) to communicate with PulseView
  * In `ESP32_LogicAnalyzer.h`, set `USE_SERIAL2_FOR_OLS` to 0
    ```C++
    #define USE_SERIAL2_FOR_OLS 0 // If 1, UART2 = OLS and UART0=Debug
    ```
  * In `ESP32_LogicAnalyzer.h`, set `OLS_Port_Baud` to 115200, in case that the ESP32 dev board cannot support 921600
    ```C++
    #if USE_SERIAL2_FOR_OLS

    #define Serial_Debug_Port Serial
    //#define Serial_Debug_Port_Baud 115200
    #define Serial_Debug_Port_Baud 921600
    //#define Serial_Debug_Port_Baud 1000000
    #define OLS_Port Serial2
    #define OLS_Port_Baud 3000000
    
    #else
    
    #define Serial_Debug_Port Serial2
    #define Serial_Debug_Port_Baud 115200
    #define OLS_Port Serial
    #define OLS_Port_Baud 115200
    
    #endif
    ```
3. Build the project and flash with PlatformIO
4. Connect PIN15 to GND (This is to disable boot messages, or PulseView cannot detect this device, probably related to [this issue](https://github.com/EUA/ESP32_LogicAnalyzer/issues/1#issuecomment-582195593))
5. After PIN15 is connected to GND, open PulseView, follow the settings in the figure
  ![image](https://github.com/CW-B-W/ESP32_LogicAnalyzer/assets/76680670/b571412e-4b85-43b3-980c-3df5d2544d7c)
6. Make sure the # of samples is at least 2K samples (or ESP32 will not respond), and run.
  ![image](https://github.com/CW-B-W/ESP32_LogicAnalyzer/assets/76680670/b26a0c52-d383-46fc-9278-f6ccbfa038ed)

## PulseView Channel -> ESP32 PIN

| Channel | PIN   |
| :-----  | ----: |
| 0 | 0  |
| 1 | 18 |
| 2 | 2  |
| 3 | 19 |
| 4 | 4  |
| 5 | 5 |
| 11 | 22 |
| 12 | 12 |
| 13 | 13 |
| 14 | 14 |
| 15 | 15 |

This can be found in the [source code](https://github.com/EUA/ESP32_LogicAnalyzer/blob/5880dabeb22ce99df7cebdd727fe66dabf98d8e3/ESP32_LogicAnalyzer.ino#L45-L61)
```C++
cfg.gpio_bus[0] = 0;
cfg.gpio_bus[1] = 18; //GPIO01 used for UART 0 RX
.
.
```
