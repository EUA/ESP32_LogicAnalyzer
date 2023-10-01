static const char* TAG = "esp32la";

#include <stdint.h>
#include "soc/i2s_struct.h"
#include "rom/lldesc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"



#define _DEBUG_MODE_x

#ifndef _DEBUG_MODE_

#define OLS_Port Serial      //Serial //Serial1 //Serial2
#define OLS_Port_Baud 921600 //115200 // 3e6

#else
#define OLS_Port Serial2     //Serial //Serial1 //Serial2
#define OLS_Port_Baud 3e6    //115200 // 3e6

#define Serial_Debug_Port Serial //Serial1 //Serial2
#define Serial_Debug_Port_Baud 921600 //115200
#endif

#define ALLOW_ZERO_RLE 0

 /// ALLOW_ZERO_RLE 1 is Fast mode.
 //Add RLE Count 0 to RLE stack for non repeated values and postpone the RLE processing so faster.
 // 8Bit Mode : ~28.4k clock per 4k block, captures 3000us while inspecting ~10Mhz clock at 20Mhz mode
 //16Bit Mode : ~22.3k clock per 4k block, captures 1500us while inspecting ~10Mhz clock at 20Mhz mode
 
 /// ALLOW_ZERO_RLE 0 is Slow mode.
 //just RAW RLE buffer. It doesn't add 0 count values for non-repeated RLE values and process flags on the fly, so little slow but efficient.
 // 8Bit Mode : ~34.7k clock per 4k block, captures 4700us while inspecting ~10Mhz clock at 20Mhz mode
 //16Bit Mode : ~30.3k clock per 4k block, captures 2400us while inspecting ~10Mhz clock at 20Mhz mode

#define CAPTURE_SIZE 128000
//#define CAPTURE_SIZE 12000
#define rle_size 96000

#define ledPin 21 //Led on while running and Blinks while transfering data.

uint32_t time_debug_indice_dma[10];
uint16_t time_debug_indice_dma_p=0;

uint32_t time_debug_indice_rle[10];
uint16_t time_debug_indice_rle_p=0;

uint16_t time_debug_indice_lenght = 10;

int stop_at_desc=-1;
unsigned int logicIndex = 0;
unsigned int triggerIndex = 0;
uint32_t readCount = CAPTURE_SIZE;
unsigned int delayCount = 0;
uint16_t trigger = 0;
uint16_t trigger_values = 0;
unsigned int useMicro = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;
boolean rleEnabled = 0;
uint32_t clock_per_read = 0;


typedef enum {
  I2S_PARALLEL_BITS_8   = 8,
  I2S_PARALLEL_BITS_16  = 16,
  I2S_PARALLEL_BITS_32  = 32,
} i2s_parallel_cfg_bits_t;

typedef struct {
  void* memory;
  size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
  int8_t gpio_bus[16];
  int8_t gpio_clk_in;
  int8_t gpio_clk_out;
  
  int clkspeed_hz;
  i2s_parallel_cfg_bits_t bits;
  i2s_parallel_buffer_desc_t* buf;
} i2s_parallel_config_t;

typedef struct {
  volatile lldesc_t* dmadesc;
  int desccount;
} i2s_parallel_state_t;

static i2s_parallel_state_t* i2s_state[2] = {NULL, NULL};

#define DMA_MAX (4096-4)

//Calculate the amount of dma descs needed for a buffer desc
static int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc) {
  int ret = (desc->size + DMA_MAX - 1) / DMA_MAX;
  return ret;
}

typedef union {
    struct {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
      };
    struct{
      uint16_t val2;
      uint16_t val1;
      };
    uint32_t val;
} dma_elem_t;

typedef enum {
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, .
     * 
     * but appears as 00 s2 00 s1, 00 s4 00 s3 at DMA buffer somehow...
     * 
     */
    
    SM_0A00_0B00 = 3,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
} i2s_sampling_mode_t;

typedef struct {
    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    bool dma_done;
    size_t dma_desc_count;
    size_t dma_desc_cur;
    int dma_desc_triggered;
    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_buf_width;
    size_t dma_sample_count;
    size_t dma_val_per_desc;
    size_t dma_sample_per_desc;
    i2s_sampling_mode_t sampling_mode;
//    dma_filter_t dma_filter;
    intr_handle_t i2s_intr_handle;
//    QueueHandle_t data_ready;
//    SemaphoreHandle_t frame_ready;
//    TaskHandle_t dma_filter_task;
} camera_state_t;

camera_state_t *s_state;

void i2s_parallel_setup( const i2s_parallel_config_t *cfg);

uint8_t channels_to_read=3;
/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK_CH_A 0xC0
#define SUMP_TRIGGER_MASK_CH_B 0xC4
#define SUMP_TRIGGER_MASK_CH_C 0xC8
#define SUMP_TRIGGER_MASK_CH_D 0xCC

#define SUMP_TRIGGER_VALUES_CH_A 0xC1
#define SUMP_TRIGGER_VALUES_CH_B 0xC5
#define SUMP_TRIGGER_VALUES_CH_C 0xC9
#define SUMP_TRIGGER_VALUES_CH_D 0xCD

#define SUMP_TRIGGER_CONFIG_CH_A 0xC2
#define SUMP_TRIGGER_CONFIG_CH_B 0xC6
#define SUMP_TRIGGER_CONFIG_CH_C 0xCA
#define SUMP_TRIGGER_CONFIG_CH_D 0xCE

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#define MAX_CAPTURE_SIZE CAPTURE_SIZE

int8_t    rle_process=-1;
uint8_t   rle_buff [rle_size];
uint8_t*  rle_buff_p;
uint8_t*  rle_buff_end;
uint8_t   rle_sample_counter;
uint32_t  rle_total_sample_counter;
uint8_t   rle_value_holder;

bool rle_init(void){
  rle_buff_p=0;
  rle_sample_counter=0;
  rle_total_sample_counter=0;
  rle_value_holder=0;
  rle_process=-1;
  
  rle_buff_p=rle_buff;
  rle_buff_end = rle_buff+rle_size-4;

  memset( rle_buff, 0x00, rle_size);
  return true;
}

void dma_serializer( dma_elem_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4
  for ( int i = 0 ; i < s_state->dma_buf_width/4 ; i++ ){
     uint8_t y =  dma_buffer[i].sample2;
     dma_buffer[i].sample2 = dma_buffer[i].sample1;
     dma_buffer[i].sample1 = y;
   }
}


void dmabuff_compresser_ch1( uint8_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4.... - > s1,s2,s3,s4,00,00,00,00...
  for ( int i=0, p=0 ; i < s_state->dma_buf_width-4 ; i+=4 ){
   dma_buffer[p++]=dma_buffer[i+1];
   dma_buffer[p++]=dma_buffer[i+3];
   }
}
void dmabuff_compresser_ch2( uint8_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4.... - > s1,s2,s3,s4,00,00,00,00...
  for ( int i=0, p=0 ; i < s_state->dma_buf_width-4 ; i+=4 ){
   dma_buffer[p++]=dma_buffer[i+0];
   dma_buffer[p++]=dma_buffer[i+2];
   }
}

void fast_rle_block_encode_asm_8bit_ch1(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU 
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */
    
   int dword_count=(sample_size/4) -1;
   
   clocka = xthal_get_ccount();
   
   /* No, Assembly is not that hard. You are just too lazzy. */
   
   // "a4"=&dma_buffer, "a5"=dword_count, "a6"=&rle_buff_p:
   
    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 2        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 0        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0          \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0:                \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end    \n" // Prepare zero-overhead loop
      "loopStart:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start:          \n"
      "l8ui a9, a4, 2        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add    \n" // if count >= 128 branch
      "rle_1_127:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127    \n" // if count >= 128 branch

      "rle_1_add:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction. 

      "rle_1_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start:          \n"
      "l8ui a9, a4, 0        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add    \n" // if count >= 128 branch
      "rle_2_127:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127    \n" // if count >= 128 branch

      "rle_2_add:            \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end:              \n"

"bltui a5, 128, rle_end_add  \n" // if count >= 128 branch
      "rle_end_127:          \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127  \n" // if count >= 128 branch

      "rle_end_add:          \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip       \n"
      "or   a5, a5, a10      \n" // 
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "rle_end_skip:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

      
    clockb = xthal_get_ccount();
    //if(time_debug_indice_rle_p < time_debug_indice_lenght)
    //  time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;

 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

}

void fast_rle_block_encode_asm_8bit_ch2(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU 
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */
    
   int dword_count=(sample_size/4) -1;
   
   clocka = xthal_get_ccount();
   
   /* No, Assembly is not that hard. You are just too lazzy. */

    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 3        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 1        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0_ch2      \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end_ch2    \n" // Prepare zero-overhead loop
      "loopStart_ch2:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_ch2:          \n"
      "l8ui a9, a4, 3        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end_ch2      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add_ch2    \n" // if count >= 128 branch
      "rle_1_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127_ch2    \n" // if count >= 128 branch

      "rle_1_add_ch2:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip_ch2         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip_ch2:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction. 

      "rle_1_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_ch2:          \n"
      "l8ui a9, a4, 1        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end_ch2      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add_ch2   \n" // if count >= 128 branch
      "rle_2_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127_ch2    \n" // if count >= 128 branch

      "rle_2_add_ch2:            \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip_ch2         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip_ch2:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_ch2:              \n"

"bltui a5, 128, rle_end_add_ch2  \n" // if count >= 128 branch
      "rle_end_127_ch2:          \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127_ch2  \n" // if count >= 128 branch

      "rle_end_add_ch2:          \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip_ch2       \n"
      "or   a5, a5, a10      \n" // 
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "rle_end_skip_ch2:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit_ch2:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

      
    clockb = xthal_get_ccount();
    
 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

}

void fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, int sample_size){ //size, not count
   //uint8_t *desc_buff_end=dma_buffer;
   //uint32_t clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU 
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : S1s1,S2s2,S3s3,S4s4
    * actual data structure of DMA memory : S2s2,S1s1,S4s4,S3s3
    */
    
   int dword_count=(sample_size/4) -1;
   
   //clocka = xthal_get_ccount();
   
   /* No, Assembly is not that hard. You are just too lazzy. */
   
    __asm__ __volatile__(
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l16ui a8, a4, 2       \n" // a8 as rle_val #2 is first   a8=&(dma_buffer+2)
      "l16ui a9, a4, 0       \n" // a9 as new_val               a9=&dma_buffer
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x8000      \n" // init rle_masks for count
      "movi a11, 0x7FFF      \n" // init rle_masks for data

"beq  a9, a8, rle_0_16       \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7FFF
#endif
      "s16i  a8, a6, 0       \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s16i  a5, a6, 2       \n" // rle_buff_p+2 = rle_counter //not needed
      "addi a6, a6, 2        \n" // rle_buff_p +=2             //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 2        \n" // rle_buff_p +=2 
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_0_16:             \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end_16 \n" // Prepare zero-overhead loop
      "loopStart_16:         \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_16:       \n"
      "l16ui a9, a4, 2       \n" // a9 as rle_val #2 is first
"beq  a9, a8, rle_1_end_16   \n" // rle_val == new_val skip

      "rle_1_127_16:         \n" //not needed 
      
      "rle_1_add_16:         \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 22.2k clocks
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // * 0x7F
      "s16i  a8, a6, 0       \n"// *rle_buff_p=rle_val;
"beqz a5, rle_1_skip_16      \n" 
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n"// *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p --
      "rle_1_skip_16:        \n" 
      "addi a6, a6, 2        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1      

      "rle_1_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_16:       \n"
      "l16ui a9, a4, 0       \n" // a9 as rle_val #0 is second
"beq  a9, a8, rle_2_end_16   \n" // rle_val == new_val continue

      "rle_2_add_16:         \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_2_skip_16      \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p -- 
      "rle_2_skip_16:        \n"
      "addi a6, a6, 2        \n" // rle_buff_p -- 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_16:      \n"

      "rle_end_add_16:       \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4 
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_end_skip_16    \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p -- 
      "rle_end_skip_16:      \n"
      "addi a6, a6, 2        \n" // rle_buff_p -- 
#endif

      "exit_16:              \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");
    
    //clockb = xthal_get_ccount();
    
    //if(time_debug_indice_rle_p < time_debug_indice_lenght)
    //time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb);
      //time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb-clocka);
    
    //Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    //Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    //Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    

/*
    Serial_Debug_Port.printf("RLE Block Output:\r\n");
    for(int i=0; i < sample_size/40 ; i++ )
      Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
    Serial_Debug_Port.println();
    */
}
