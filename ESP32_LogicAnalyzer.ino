
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
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.begin(Serial_Debug_Port_Baud);
  //Using for development
  //OLS_Port.begin(OLS_Port_Baud, SERIAL_8N1, 13, 12);
  #endif
  
  OLS_Port.begin(OLS_Port_Baud);

  //WiFi.mode(WIFI_OFF);
  //btStop();
    
  pinMode(ledPin, OUTPUT);

  dma_desc_init(CAPTURE_SIZE);

  cfg.gpio_bus[0]  = 0;
  cfg.gpio_bus[1]  = 32;//GPIO01 used for UART 0 RX, able to use it if you select different UART port (1,2) as OLS_Port
  cfg.gpio_bus[2]  = 2;
  cfg.gpio_bus[3]  = 33;//GPIO03 used for UART 0 TX
  cfg.gpio_bus[4]  = 4;
  cfg.gpio_bus[5]  = 5;
  cfg.gpio_bus[6]  = 26; //GPIO06 used for SCK, bootloop, //GPIO16 is UART2 RX 
  cfg.gpio_bus[7]  = 27; //GPIO07 used for SDO, bootloop  //GPIO17 is UART2 TX
  
  cfg.gpio_bus[8]  = 18;//GPIO8 used for SDI, bootloop
  cfg.gpio_bus[9]  = 19;//GPIO9 lead SW_CPU_RESET on WROOVER module
  cfg.gpio_bus[10] = 20;//GPI10 lead SW_CPU_RESET on WROOVER module
  cfg.gpio_bus[11] = 21;//GPIO11 used for CMD, bootloop
  cfg.gpio_bus[12] = 12;
  cfg.gpio_bus[13] = 13;
  cfg.gpio_bus[14] = 14;
  cfg.gpio_bus[15] = 15;
  
  cfg.gpio_clk_out= 22; // Pin22 used for LedC output
  cfg.gpio_clk_in = 23; // Pin23 used for XCK input from LedC
  
  //GPIO 24,28,29,30,31 results bootloop
    
  //cfg.bits = I2S_PARALLEL_BITS_8; //not implemented yet...
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
  vTaskDelay(1); //To avoid WDT
  
  if (OLS_Port.available() > 0) {
    //int z = OLS_Port.available();
    cmdByte = OLS_Port.read();
    
    #ifdef _DEBUG_MODE_
    Serial_Debug_Port.printf("CMD: 0x%02X\r\n", cmdByte);
    #endif
    
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
        #ifdef _DEBUG_MODE_
        if (trigger) {
          Serial_Debug_Port.printf("Trigger Set for inputs : ");
          for ( int i = 0; i < 16 ; i++ )
            if (( trigger >> i) & 0x1 )
              Serial_Debug_Port.printf("%d,  ", i );
          Serial_Debug_Port.println();
        }
        #endif
        break;
      case SUMP_TRIGGER_VALUES_CH_A:
        getCmd();

        trigger_values = ((uint16_t)cmdBytes[1] << 8 ) | cmdBytes[0];
        #ifdef _DEBUG_MODE_
        if (trigger) {
          Serial_Debug_Port.printf("Trigger Val for inputs : ");
          for ( int i = 0; i < 16 ; i++ )
            if (( trigger >> i) & 0x1 )
              Serial_Debug_Port.printf("%C,  ", (( trigger_values >> i ) & 0x1 ? 'H' : 'L') );
          Serial_Debug_Port.println();
        }
        #endif
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
        channels_to_read = (~(cmdBytes[0] >> 2) & 0x0F);
        
        #ifdef _DEBUG_MODE_
        if (rleEnabled)
          Serial_Debug_Port.println("RLE Compression enable");
        else
          Serial_Debug_Port.println("Non-RLE Operation");

        Serial_Debug_Port.printf("Demux %c\r\n", cmdBytes[0] & 0x01 ? 'Y' : 'N');
        Serial_Debug_Port.printf("Filter %c\r\n", cmdBytes[0] & 0x02 ? 'Y' : 'N');
        Serial_Debug_Port.printf("Channels to read: 0x%X \r\n",  channels_to_read);
        Serial_Debug_Port.printf("External Clock %c\r\n", cmdBytes[0] & 0x40 ? 'Y' : 'N');
        Serial_Debug_Port.printf("inv_capture_clock %c\r\n", cmdBytes[0] & 0x80 ? 'Y' : 'N');
        #endif
        break;

      case SUMP_GET_METADATA:
        get_metadata();
        break;
      case SUMP_SELF_TEST:
        break;
      default:
        #ifdef _DEBUG_MODE_
        Serial_Debug_Port.printf("Unrecognized cmd 0x%02X\r\n", cmdByte );
        #endif
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
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("CMDs ");
  for (int q = 0; q < 4; q++) {
    Serial_Debug_Port.printf(" 0x%02X", cmdBytes[q]);
  }
  Serial_Debug_Port.println();
  #endif
}

void get_metadata() {
  /* device name */
  OLS_Port.write((uint8_t)0x01);
  //OLS_Port.write("AGLAMv0");
  OLS_Port.write("ESP32 Logic Analyzer v0.31");
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
  uint32_t capture_speed = 20e6;
  OLS_Port.write((uint8_t)0x23);
  OLS_Port.write((uint8_t) (capture_speed >> 24) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 16) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 8) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 0) & 0xFF);

  /* number of probes */
  OLS_Port.write((uint8_t)0x40);
  //OLS_Port.write((uint8_t)0x08);//8
  //OLS_Port.write((uint8_t)0x10);//16
  //OLS_Port.write((uint8_t)0x20);//32
  OLS_Port.write((uint8_t) cfg.bits);

  /* protocol version (2) */
  OLS_Port.write((uint8_t)0x41);
  OLS_Port.write((uint8_t)0x02);

  /* end of data */
  OLS_Port.write((uint8_t)0x00);
}

void setupDelay() {
  double rate = 100000000.0 / (divider + 1.0);
  enable_out_clock((int)rate);
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("Capture Speed : %.2f Mhz\r\n", rate/1000000.0);
  #endif
}

void captureMilli() {
  uint32_t a, b, c, d;
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("FreeHeap         :%u\r\n", ESP.getFreeHeap());
  Serial_Debug_Port.printf("FreeHeap 64 Byte :%u\r\n", heap_caps_get_largest_free_block(64) );
  Serial_Debug_Port.printf("Triger Values 0x%X\r\n", trigger_values);
  Serial_Debug_Port.printf("Triger        0x%X\r\n", trigger);
  Serial_Debug_Port.printf("Running on CORE #%d\r\n", xPortGetCoreID());
  Serial_Debug_Port.printf("Reading %d Samples\r\n", readCount);
  #endif
  digitalWrite( ledPin, HIGH );

  ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
  rle_init();
  
  start_dma_capture();
  yield();
  I2S0.conf.rx_start = 1;
  delay(100); //this delay is strictly need for error free capturing...
  
  while (! s_state->dma_done )
    delay(100);
  
  yield();

  digitalWrite( ledPin, LOW );
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("ReadCount:  %d complete\r\n",readCount);
  Serial_Debug_Port.printf("DMA Desc Current: %d\r\n",  s_state->dma_desc_cur);
  #endif
  ESP_LOGD(TAG, "Copying buffer.");

  int filled_desc = ceil( (readCount/2.0) / s_state->dma_sample_per_desc);
  int filled_sample_offset = ((readCount/2) % s_state->dma_sample_per_desc); //((readCount - 1) % s_state->dma_val_per_desc) % s_state->dma_sample_per_desc;
  int filled_full_sample_offset = s_state->dma_sample_per_desc;
  int tx_count = 0;
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("used_desc = %d\r\n", filled_desc);
  Serial_Debug_Port.printf("used_sample_offset = %d\r\n", filled_sample_offset);

  Serial_Debug_Port.printf( "\r\nDMA Times:" );
  for(int i = 0 ; i <  time_debug_indice_lenght ; i++){
    Serial_Debug_Port.printf( "%u\t", time_debug_indice_dma[i]-time_debug_indice_dma[0] );
    }
  
  Serial_Debug_Port.printf( "\r\nRLE Times:" );
  for(int i = 0 ; i <  time_debug_indice_lenght ; i++){
    Serial_Debug_Port.printf( "%u\t", time_debug_indice_rle[i]-time_debug_indice_dma[0] );
    //Serial_Debug_Port.printf( "%u\t", time_debug_indice_rle[i]-time_debug_indice_dma[0] );
    }
  
  Serial_Debug_Port.printf( "\r\nDone\r\n" );
  Serial_Debug_Port.flush();
  #endif
  
  filled_desc--;
  filled_full_sample_offset--;
  if( filled_sample_offset-- == 0)
    filled_sample_offset = filled_full_sample_offset;

  dma_elem_t cur;

/*
  Serial_Debug_Port.printf("\r\nRAW BlocX \r\n");
  for ( int i = 0 ; i < 100 ; i++ ){
     cur = (dma_elem_t&)s_state->dma_buf[0][i];
     Serial_Debug_Port.printf("0x%X, ", cur.sample2);
     Serial_Debug_Port.printf("0x%X, ", cur.sample1);
   }

  Serial_Debug_Port.printf("\r\nRAW Block InpuX:\r\n");
  for ( int i = 0 ; i < 400 ; i+=2 ){
     uint8_t *crx = (uint8_t*)s_state->dma_buf[0];
     Serial_Debug_Port.printf("0x%X, ", *(crx+i) );
     }
   Serial_Debug_Port.println();
*/


  if(s_state->dma_desc_triggered < 0){ //if not triggered mode,
    s_state->dma_desc_triggered=0;  //first desc is 0
    ESP_LOGD(TAG, "Normal TX");
    }
  else
    ESP_LOGD(TAG, "Triggered TX");

  //At SUMP protocol, you need to replay cache in LIFO, not FIFO... So send samples from end, not from start...
  
  if(rleEnabled){
    ESP_LOGD(TAG, "RLE TX");
    int rle_fill = (rle_buff_p - rle_buff);
    #ifdef _DEBUG_MODE_
    Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", rle_fill );
    #endif
    uint32_t rle_sample_count=0;
    uint32_t rle_sample_req_count=0;
    
    for(int i=0; i<rle_fill ; i++){
#if ALLOW_ZERO_RLE
      if( i%2 != 0 ){
        rle_sample_count += rle_buff[i];
        if( readCount > rle_sample_count )
          rle_sample_req_count= i;
        }

#else
#endif
    }
    #ifdef _DEBUG_MODE_
    Serial_Debug_Port.printf( "Total RLE Sample Count = %d\r\n", rle_sample_count ); 
    Serial_Debug_Port.printf( "Total RLE Sample Req Count = %d\r\n", rle_sample_req_count );
    #endif
/*
    for(int i=0; i<32 ; i++){
     Serial_Debug_Port.printf("Processing DMA Desc: %d", i);
     fast_rle_block_encode_asm( (uint8_t*)s_state->dma_buf[i], s_state->dma_buf_width);
     if( rle_buff_p-rle_buff > rle_size - 4000 )
      break;
     }
*/   

    if( channels_to_read == 3 )
    {
      int a=0;
      #ifdef _DEBUG_MODE_
      Serial_Debug_Port.printf("Debug RLE BUFF:" );
      for( int i=0; i <50 ; i++){
        Serial_Debug_Port.printf("0x%X ", rle_buff[i] );
        }
      Serial_Debug_Port.printf("\r\n" );
      #endif
      
#if ALLOW_ZERO_RLE
      for( int i =  rle_fill-4;  i>=0 ; i-=4  ){
      //for( int i =  rle_sample_req_count;  i>=0 ; i-=2  ){
        //if( rle_buff[i+1] !=0 )
       
        OLS_Port.write( rle_buff[i+2] | 0x00 ); //Count sent first
        OLS_Port.write( rle_buff[i+3] | 0x80 ); //Count sent later
        OLS_Port.write( rle_buff[i+0] & 0xFF ); //Value sent first
        OLS_Port.write( rle_buff[i+1] & 0x7F ); //Value sent later
        }
#else
      for( int i = rle_fill-2;  i>=0 ; i-=2  ){
        OLS_Port.write( rle_buff[i] );
        OLS_Port.write( rle_buff[i+1] );
        }

#endif
      }

      else{
//The buffer need to send from end to start due OLS protocol...
#if ALLOW_ZERO_RLE
      //for( int i =  0 ;  i < readCount - rle_sample_req_count ; i++  )
      //  OLS_Port.write( 0 );
      
      for( int i =  rle_fill-2;  i>=0 ; i-=2  ){
      //for( int i =  rle_sample_req_count;  i>=0 ; i-=2  ){
        if( rle_buff[i+1] !=0 )
        OLS_Port.write( rle_buff[i+1] | 0x80 ); //Count sent first
        OLS_Port.write( rle_buff[i+0] & 0x7F ); //Value sent later
        }
#else
      //for( int i =  (rle_buff_p - rle_buff)-1;  i>=0 ; i--  ){
      for( int i = rle_fill-1;  i>=0 ; i--  ){
        OLS_Port.write( rle_buff[i] );
        }

#endif
      }
      
    
/*

    for( int i = 0  ;  i < 200  ; i++  )
     OLS_Port.write( 8 );
    
    for( int i = s_state->dma_buf_width ;  i > 0   ; i-=4  ){
       //OLS_Port.write( *(((uint8_t*)s_state->dma_buf[0])+i) & 0x7F );
       uint8_t *crx = (uint8_t*)s_state->dma_buf[0];
       OLS_Port.write( *(crx+i+0) );
       OLS_Port.write( *(crx+i+2) );
     }
*/   
    /*
    for( int i = (rle_buff_p - rle_buff)-2; i > 0  ; i-=2  ){
      OLS_Port.write( rle_buff[i+1] -1 | 0x80 ) ;
      OLS_Port.write( rle_buff[i+0] & 0x7F ) ;
      }
      */
    /*
    for( int i = 0  ;  i < 200  ; i++  )
     OLS_Port.write( 8 );

    
    for ( int i =  filled_sample_full_offset ; i >= 0 ; i-- ) {
        cur = s_state->dma_buf[0][i];
        if (channels_to_read == 1) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.sample1);
        }
        }
    */

    /*
    int x0 = readCount - (rle_buff_p-rle_buff)/2;
    for( int i = 0; i < x0 ; i+=2 ){
      OLS_Port.write( ((i/2)%2 ) | 0x80 );
      OLS_Port.write( (i/2)%2 );     
    }
*/      

    }
  else{
    for ( int j = filled_desc; j >= 0 ; j-- ) {
      ESP_LOGD(TAG, "filled_buff trgx = %d", (j + s_state->dma_desc_triggered + s_state->dma_desc_count) % s_state->dma_desc_count);
      digitalWrite( ledPin, !digitalRead(ledPin) );
      //for( int i=s_state->dma_buf_width/4 - 1; i >=0 ; i-- ){
      
      if(0){
        j == filled_desc ? filled_sample_offset : filled_full_sample_offset;
        //int descnum=j +s_state->dma_desc_triggered +s_state->dma_desc_count) % s_state->dma_desc_count][i];
        
        //dmabuff_compresser_ch1( s_state->dma_buf[descnum] );
        
        //OLS_Port.write(s_state->dma_buf[descnum]);
        
        }
      else
      for ( int i = (j == filled_desc ? filled_sample_offset : filled_full_sample_offset ) ; i >= 0 ; i-- ) {
        //Serial.printf( "%02X %02X ", s_state->dma_buf[j][i].sample1,  s_state->dma_buf[j][i].sample2 );
        cur = s_state->dma_buf[ (j
                                +s_state->dma_desc_triggered
                                +s_state->dma_desc_count) % s_state->dma_desc_count][i];
        
        //uint8_t* cx = (uint8_t*)&cur.val;
        //ESP_LOGD(TAG, "cur [0-4]    %02X %02X %02X %02X", cx[0],cx[1],cx[2],cx[3] );
        //ESP_LOGD(TAG, "cur u1s1u2s2 %02X %02X %02X %02X", cur.unused1,cur.sample1,cur.unused2,cur.sample2 );
        
        if (channels_to_read == 1) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.sample1);
          OLS_Port.flush();
          yield();
        }
        else if (channels_to_read == 2) {
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.unused1);
          OLS_Port.flush();
          yield();
        }
        else if (channels_to_read == 3)
        {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.sample1);
          OLS_Port.write(cur.unused1);
          OLS_Port.flush();
          yield();
        }
        tx_count += 2;
        //vTaskDelay(1);
        if (tx_count >= readCount){
          #ifdef _DEBUG_MODE_
            Serial_Debug_Port.printf("BRexit triggered." );
           #endif
          goto brexit;
          }
      }
    vTaskDelay(1);
    }
    
  }
brexit:
  //ESP_LOGD(TAG, "TX_Count: %d", tx_count);
  ESP_LOGD(TAG, "End. TX: %d", tx_count);
  digitalWrite( ledPin, LOW );
}
