/*************************************************************************
 * 
 *  ESP32 Logic Analyzer
 *  Copyright (C) 2020 Erdem U. Altinyurt
 *    
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *************************************************************************
 *
 *  This project steals some code from 
 *  https://github.com/igrr/esp32-cam-demo for I2S DMA
 *  and
 *  https://github.com/gillham/logic_analyzer/issues for SUMP protocol as template.
 * 
 */ 

#include "ESP32_LogicAnalyzer.h"

i2s_parallel_buffer_desc_t bufdesc;
i2s_parallel_config_t cfg;

void setup(void) {
  Serial_Debug_Port.begin(Serial_Debug_Port_Baud);
  OLS_Port.begin(OLS_Port_Baud);
  
  pinMode(ledPin, OUTPUT);

  dma_desc_init(CAPTURE_SIZE);

  cfg.gpio_bus[0] = 0;
  cfg.gpio_bus[1] = -1;//1 UART 0 RX
  cfg.gpio_bus[2] = 2;
  cfg.gpio_bus[3] = -1;//3 UART 0 TX
  cfg.gpio_bus[4] = 4;
  cfg.gpio_bus[5] = 5;
  cfg.gpio_bus[6] = -1;//6 bootloop SCK
  cfg.gpio_bus[7] = -1;//7 bootloop SDO

  cfg.gpio_bus[8] = -1;//8 bootloop SDI
  cfg.gpio_bus[9] = 9;
  cfg.gpio_bus[10] = 10;
  cfg.gpio_bus[11] = -1;//11 bootloop CMD
  cfg.gpio_bus[12] = 12;
  cfg.gpio_bus[13] = 13;
  cfg.gpio_bus[14] = 14;
  cfg.gpio_bus[15] = 15;

  cfg.gpio_clk = 23; // Pin23 used for XCK input from LedC
  cfg.bits = I2S_PARALLEL_BITS_16;
  cfg.clkspeed_hz = 2 * 1000 * 1000; //resulting pixel clock = 1MHz
  cfg.buf = &bufdesc;

  //enable_out_clock(I2S_HZ);
  //fill_dma_desc( bufdesc );
  i2s_parallel_setup(&cfg);
}

void captureMilli(void);
void getCmd(void);
void blinkled(void);
void get_metadata(void);

int cmdByte = 0;
byte cmdBytes[5];

void loop()
{
  int i;
  if (OLS_Port.available() > 0) {
    int z = OLS_Port.available();
    cmdByte = OLS_Port.read();
    Serial_Debug_Port.printf("CMD: 0x%02X\r\n", cmdByte);
    int chan_num = 0;
    switch (cmdByte) {
      case SUMP_RESET:
        break;
      case SUMP_QUERY:
        OLS_Port.print(F("1ALS"));
        //OLS_Port.print(F("1SLO"));
        break;
      case SUMP_ARM:
        captureMilli();
        break;
      case SUMP_TRIGGER_MASK_CH_A:
        getCmd();
        trigger = ((uint16_t)cmdBytes[1] << 8 ) | cmdBytes[0];
        if (trigger) {
          Serial_Debug_Port.printf("Trigger Set for inputs : ");
          for ( int i = 0; i < 16 ; i++ )
            if (( trigger >> i) & 0x1 )
              Serial_Debug_Port.printf("%d,  ", i );
          Serial_Debug_Port.println();
        }
        break;
      case SUMP_TRIGGER_VALUES_CH_A:
        getCmd();

        trigger_values = ((uint16_t)cmdBytes[1] << 8 ) | cmdBytes[0];
        if (trigger) {
          Serial_Debug_Port.printf("Trigger Val for inputs : ");
          for ( int i = 0; i < 16 ; i++ )
            if (( trigger >> i) & 0x1 )
              Serial_Debug_Port.printf("%C,  ", (( trigger_values >> i ) & 0x1 ? 'H' : 'L') );
          Serial_Debug_Port.println();
        }
        break;

      case SUMP_TRIGGER_MASK_CH_B:
      case SUMP_TRIGGER_MASK_CH_C:
      case SUMP_TRIGGER_MASK_CH_D:
      case SUMP_TRIGGER_VALUES_CH_B:
      case SUMP_TRIGGER_VALUES_CH_C:
      case SUMP_TRIGGER_VALUES_CH_D:
      case SUMP_TRIGGER_CONFIG_CH_A:
      case SUMP_TRIGGER_CONFIG_CH_B:
      case SUMP_TRIGGER_CONFIG_CH_C:
      case SUMP_TRIGGER_CONFIG_CH_D:
        getCmd();
        /*
           No config support
        */
        break;
      case SUMP_SET_DIVIDER:
        /*
             the shifting needs to be done on the 32bit unsigned long variable
           so that << 16 doesn't end up as zero.
        */
        getCmd();
        divider = cmdBytes[2];
        divider = divider << 8;
        divider += cmdBytes[1];
        divider = divider << 8;
        divider += cmdBytes[0];
        setupDelay();
        break;
      case SUMP_SET_READ_DELAY_COUNT:
        getCmd();
        readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
        if (readCount > MAX_CAPTURE_SIZE)
          readCount = MAX_CAPTURE_SIZE;
        delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
        if (delayCount > MAX_CAPTURE_SIZE)
          delayCount = MAX_CAPTURE_SIZE;
        break;

      case SUMP_SET_FLAGS:
        getCmd();
        rleEnabled = cmdBytes[1] & 0x1;
        if (rleEnabled)
          Serial_Debug_Port.println("RLE Compression enable");
        else
          Serial_Debug_Port.println("Non-RLE Operation enable");

        Serial_Debug_Port.printf("Demux %c\r\n", cmdBytes[0] & 0x01 ? 'Y' : 'N');
        Serial_Debug_Port.printf("Filter %c\r\n", cmdBytes[0] & 0x02 ? 'Y' : 'N');
        channels_to_read = (~(cmdBytes[0] >> 2) & 0x0F);
        Serial_Debug_Port.printf("Channels to read: 0x%X \r\n",  channels_to_read);
        Serial_Debug_Port.printf("External Clock %c\r\n", cmdBytes[0] & 0x40 ? 'Y' : 'N');
        Serial_Debug_Port.printf("inv_capture_clock %c\r\n", cmdBytes[0] & 0x80 ? 'Y' : 'N');
        break;

      case SUMP_GET_METADATA:
        get_metadata();
        break;
      case SUMP_SELF_TEST:
        break;
      default:
        Serial_Debug_Port.printf("Unrecognized cmd 0x%02X\r\n", cmdByte );
        getCmd();
        break;
    }
  }
}


void getCmd() {
  delay(10);
  cmdBytes[0] = OLS_Port.read();
  cmdBytes[1] = OLS_Port.read();
  cmdBytes[2] = OLS_Port.read();
  cmdBytes[3] = OLS_Port.read();
  Serial_Debug_Port.printf("CMDs ");
  for (int q = 0; q < 4; q++) {
    Serial_Debug_Port.printf(" 0x%02X", cmdBytes[q]);
  }
  Serial_Debug_Port.println();
}


void get_metadata() {
  /* device name */
  OLS_Port.write((uint8_t)0x01);
  //OLS_Port.write("AGLAMv0");
  OLS_Port.write("ESP32 Logic Analyzer v0.1");
  OLS_Port.write((uint8_t)0x00);

  /* firmware version */
  OLS_Port.write((uint8_t)0x02);
  OLS_Port.print("0.10");
  OLS_Port.write((uint8_t)0x00);

  /* sample memory */
  OLS_Port.write((uint8_t)0x21);
  uint32_t capture_size = CAPTURE_SIZE;
  OLS_Port.write((uint8_t) (capture_size >> 24) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 16) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 8) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 0) & 0xFF);

  /* sample rate (20MHz) */
  OLS_Port.write((uint8_t)0x23);
  OLS_Port.write((uint8_t)0x01);//20Mhz
  OLS_Port.write((uint8_t)0x31);
  OLS_Port.write((uint8_t)0x2d);
  OLS_Port.write((uint8_t)0x00);
  /*
    OLS_Port.write((uint8_t)0x23);
    OLS_Port.write((uint8_t)0x02);//40Mhz
    OLS_Port.write((uint8_t)0x62);
    OLS_Port.write((uint8_t)0x5A);
    OLS_Port.write((uint8_t)0x00);
  */

  /* number of probes */
  OLS_Port.write((uint8_t)0x40);
  //OLS_Port.write((uint8_t)0x08);//8
  OLS_Port.write((uint8_t)0x10);  //16
  //OLS_Port.write((uint8_t)0x20);//32

  /* protocol version (2) */
  OLS_Port.write((uint8_t)0x41);
  OLS_Port.write((uint8_t)0x02);

  /* end of data */
  OLS_Port.write((uint8_t)0x00);
}

void setupDelay() {
  double rate = 100000000.0 / (divider + 1.0);
  double cpuclk = 240000000;
  clock_per_read = cpuclk / rate;
  delayMicro = 1000000 / rate;

  //enable_out_clock(10*1000*1000);
  int rateint = rate;
  //int rateint = 40000000;
  enable_out_clock(rateint);

  Serial_Debug_Port.printf("Rate=%f\r\n", rate);
  Serial_Debug_Port.printf("DelayMicro=%u\r\n", delayMicro);
  Serial_Debug_Port.printf("Mhz : %u\r\n", rate);
  Serial_Debug_Port.printf("Divider: %u\r\n", divider);
  Serial_Debug_Port.printf("Clock_per_read:%u\r\n", clock_per_read);
}

void captureMilli() {
  uint32_t a, b, c, d;
  Serial_Debug_Port.printf("FreeHeap             :%u\r\n", ESP.getFreeHeap());
  Serial_Debug_Port.printf("FreeHeap Maximum Larg:%u\r\n", heap_caps_get_largest_free_block(1) );
  Serial_Debug_Port.printf("FreeHeap Maximum Lar2:%u\r\n", heap_caps_get_largest_free_block(100) );
  Serial_Debug_Port.printf("FreeHeap After Malloc:%u\r\n", ESP.getFreeHeap());
  Serial_Debug_Port.printf("Triger Values 0x%X\r\n", trigger_values);
  Serial_Debug_Port.printf("Triger        0x%X\r\n", trigger);
  Serial_Debug_Port.printf("Running on CORE #%d\r\n", xPortGetCoreID());
  Serial_Debug_Port.printf("Reading %d Samples\r\n", readCount);

  digitalWrite( ledPin, HIGH );

  //CAPTURE_SIZE = readCount;
  //dma_desc_deinit();
  //dma_desc_init(readCount);

  ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);

  /*
    if(readCount*2 < 4000){
      lldesc_t* pd = &s_state->dma_desc[0];
      pd->length = readCount*2;
      pd->size = pd->length;
      pd->owner = 1;
      pd->sosf = 1;
      pd->eof = 1;
      pd->qe.stqe_next = 0x0;
      //pd->qe.stqe_next = &s_state->dma_desc[1];
      //I2S0.rx_eof_num = readCount;
      //I2S0.in_link.addr = (uint32_t) pd;
      }
  */
  
  start_dma_capture();

  while (! s_state->dma_done )
    delay(100);

  yield();

  digitalWrite( ledPin, LOW );

  Serial_Debug_Port.printf("ClockPerRead:%f\r\n", (b - a - 1) / (double)readCount);
  Serial_Debug_Port.printf("Clock Diff:  %d\r\n", b - a - 1);
  Serial_Debug_Port.printf("First Clock: %u\r\n", a);
  Serial_Debug_Port.printf("Last  Clock: %u\r\n", b);
  ESP_LOGD(TAG, "Copying buffer.");

  int filled_buff = (readCount - 1) / s_state->dma_val_per_desc;
  int filled_sample_offset = (((readCount - 1) % s_state->dma_val_per_desc) / 2) % s_state->dma_sample_per_desc;
  int filled_sample_full_offset = s_state->dma_buf_width / 4 - 1;
  int tx_count = 0;
  Serial_Debug_Port.printf("filled_buff = %d\r\n", filled_buff);
  Serial_Debug_Port.printf("filled_buff_offset = %d\r\n", filled_sample_offset);
  dma_elem_t cur;

  if(s_state->dma_desc_triggered<0){ //if not triggered mode,
    s_state->dma_desc_triggered=0;  //first desc is 0
    ESP_LOGD(TAG, "Normal TX");
    }
  else
    ESP_LOGD(TAG, "Triggered TX");
  {
    for ( int j = filled_buff; j >= 0 ; j-- ) {
      ESP_LOGD(TAG, "filled_buff trgx = %d", (j + s_state->dma_desc_triggered+s_state->dma_desc_count) % s_state->dma_desc_count);
      digitalWrite( ledPin, !digitalRead(ledPin) );
      //for( int i=s_state->dma_buf_width/4 - 1; i >=0 ; i-- ){
      for ( int i = (j == filled_buff ? filled_sample_offset : filled_sample_full_offset ) ; i >= 0 ; i-- ) {
        //Serial.printf( "%02X %02X ", s_state->dma_buf[j][i].sample1,  s_state->dma_buf[j][i].sample2 );
        cur = s_state->dma_buf[(j + s_state->dma_desc_triggered+s_state->dma_desc_count) % s_state->dma_desc_count][i];
        if (channels_to_read == 1) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.sample1);
        }
        else if (channels_to_read == 2) {
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.unused1);
        }
        else if (channels_to_read == 3) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.sample1);
          OLS_Port.write(cur.unused1);
        }
        tx_count += 2;
        if (tx_count >= readCount)
          goto brexit;
      }
      //buff_process_trigger((uint16_t*)s_state->dma_buf[j], s_state->dma_buf_width);
    }
  }
brexit:
  //OLS_Port.flush();
  //ESP_LOGD(TAG, "TX_Count: %d", tx_count);
  ESP_LOGD(TAG, "End. TX: %d", tx_count);
  digitalWrite( ledPin, LOW );
}
