
#include <stdint.h>
#include "soc/i2s_struct.h"
#include "config.h"
#include "rom/lldesc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include "driver/ledc.h"

#define USE_SERIAL2_FOR_OLS 1 // If 1, UART2 = OLS and UART0=Debug

#define ALLOW_ZERO_RLE 1 // 1 Fast ~26k clock per 4k block, 0 SLOW ~32k clock per 4k block

#define CAPTURE_SIZE 128000
//#define CAPTURE_SIZE 12000
#define rle_size 96000

#define ledPin 21 //Led on while running and Blinks while transfering data.




#if USE_SERIAL2_FOR_OLS

#define Serial_Debug_Port Serial
//#define Serial_Debug_Port_Baud 115200
#define Serial_Debug_Port_Baud 921600
//#define Serial_Debug_Port_Baud 1000000
#define OLS_Port Serial2
#define OLS_Port_Baud 3000000

#else

#define Serial_Debug_Port Serial2
#define Serial_Debug_Port_Baud 921600
#define OLS_Port Serial
#define OLS_Port_Baud 921600

#endif


unsigned int time_debug_indice_dma[1024];
unsigned int time_debug_indice_dma_p=0;

unsigned int time_debug_indice_rle[1024];
unsigned int time_debug_indice_rle_p=0;



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
  int gpio_bus[24];
  int gpio_clk;
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
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
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


#define USE_TX2_FOR_OLS

#define CHANPIN GPIO.in

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

int8_t rle_process=-1;
uint8_t rle_buff [rle_size];
uint8_t* rle_buff_p;
uint8_t* rle_buff_end;
uint8_t rle_sample_counter;
uint32_t rle_total_sample_counter;
uint8_t rle_value_holder;

bool rle_init(void){
  rle_buff_p=0;
  rle_sample_counter=0;
  rle_total_sample_counter=0;
  rle_value_holder=0;
  rle_process=-1;
  
  rle_buff_p=rle_buff;
  rle_buff_end = rle_buff+rle_size-4;

  memset( rle_buff, 0, rle_size);
}

void dma_serializer( dma_elem_t *dma_buffer ){
  for ( int i = 0 ; i < s_state->dma_buf_width/4 ; i++ ){
     uint8_t y =  dma_buffer[i].sample2;
     dma_buffer[i].sample2 = dma_buffer[i].sample1;
     dma_buffer[i].sample1 = y;
   }
}
/*
void fast_rle_block_encode_asm(uint8_t *dma_buffer, int sample_size){
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0,clockb=0;

   Serial_Debug_Port.printf("*dma_buffer = %p\r\n",dma_buffer);
   Serial_Debug_Port.printf("*rle_buffer = %p\r\n",rle_buff);

   
   Serial_Debug_Port.printf("RAW Block Input:\r\n");
   for(int i = 0; i < sample_size/40 ; i+=2 )
     Serial_Debug_Port.printf("0x%X, ", dma_buffer[i]);
   Serial_Debug_Port.println();
   
   //delay(10);
    __asm__ __volatile__(
      "rsr.ccount %0         \n" // Read clock #1
      "movi a7, 0x7F         \n"
      "l32i a4, %2, 0        \n" // Load store dma_buffer address
      "mov  a5, a4           \n" // Load store dma_buffer_p address
      "l32i a6, %4, 0        \n" // Load store rle_buffer address

      "l8ui a8, a4, 0        \n" // a8 as rle_val
      "loopnez %3, rle_end   \n" // Prepare zero-overhead loop
      "loopStart:            \n"
      "addi a4, a4, 2        \n" // increase dma_buffer_p pointer by 2
      "l8ui a9, a4, 0        \n" // a9 as new_val
      "beq  a9, a8, rle_cont \n" // rle_val == new_val continue

      "rle_add:              \n"
      "sub  a10, a4, a5      \n" // count (a10) = dma_buffer_p (end) - dma_buffer_p (start) 
      "add  a5, a5, a10      \n" // move dma_buff to val
      //"movi a10, 129  \n"
      //"bgeui a10, 128, rle_127 \n" // if count >= 128 branch
      "bgeui a10, 256, rle_127 \n" // if count >= 128 branch

      "rle_add_remaining:    \n" // if count >= 128 branch 
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "srli a10, a10,1       \n" // Divide by 2, because of 16bit DMA
      "s8i a10, a6, 1        \n" // *rle_buff_p+1 = count;   /2 ??
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_cont:             \n"
      "nop                   \n"
      "rle_end:              \n"
      
      "l32i a7, %4, 0        \n" // reload RLE buff
      "beq  a6, a7, rle_add  \n" // if rle_buff (start) == lre_buff_p add to rle_buff
      "j exit                \n"

      "rle_127:              \n"
      "movi a7, 127          \n"
      "s8i  a8, a6, 0        \n" // *rle_buff_p = rle_val;
      "s8i  a7, a6, 1        \n" // *rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      //"addi a10, a10, -127   \n" // count=-127
      "addi a10, a10, -256   \n" // count=-127
      //"bgeui a10, 128, rle_127 \n" // if count >= 128 branch
      "bgeui a10, 256, rle_127 \n" // if count >= 128 branch
      "j rle_add_remaining   \n"

      "exit:                 \n"
      "s32i a6, %4, 0        \n" // update rle_buff_p value
      "rsr.ccount %1         \n" // Read clock #2

      : "=a"(clocka), "=a"(clockb) :"a"(&dma_buffer), "a"(sample_size/2), "a"(&rle_buff_p): "a4","a5","a6","a7","a8","a9","a10", "memory");

    delay(1);
    Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    Serial_Debug_Port.printf( "RLE Buffer = %d\r\n", (rle_buff_p - rle_buff) );

    //Serial_Debug_Port.printf( "ASM A = %d\r\n", clocka );

   Serial_Debug_Port.printf("RLE Block Output:\r\n");
   for(int i=0; i < sample_size/20 ; i++ )
     Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
   Serial_Debug_Port.println();

}
*/



inline void fast_rle_block_encode_asm_8bit(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU 
    * for capture 20Mhz sampling speed.
    */
/*
   Serial_Debug_Port.printf("RAW Block Input:\r\n");
   for(int i = 0; i < sample_size/40 ; i+=4 ){
     Serial_Debug_Port.printf("0x%X, ", dma_buffer[i+2]);
     Serial_Debug_Port.printf("0x%X, ", dma_buffer[i]);
     }
   Serial_Debug_Port.println();
*/
   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */
    
   int dword_count=(sample_size/4) -1;
   
   clocka = xthal_get_ccount();
   
   /* No, Assembly is not that hard. You are just too lazzy. */

    __asm__ __volatile__(
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
      "l32i a6, %2, 0        \n" // Load store rle_buffer address

      "l8ui a8, a4, 2        \n" // a8 as rle_val #2 is first
      "l8ui a9, a4, 0        \n" // a9 as new_val
      "beq  a9, a8, rle_0    \n" // rle_val == new_val skip

      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      //"movi a5, 0            \n" //                        *
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = rle_counter; *
      "movi a5, -1           \n" // rle_counter=-1         *
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      //"addi a6, a6, 1        \n" // rle_buff_p ++
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0:                \n"
      "addi a5, a5, 1        \n" // rle_counter++
   "loopnez %1, rle_loop_end \n" // Prepare zero-overhead loop
      "loopStart:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start:                \n"
      "l8ui a9, a4, 2        \n" // a8 as rle_val #2 is first
      "beq  a9, a8, rle_1_end\n" // rle_val == new_val skip

      "bltui a5, 128, rle_1_add \n" // if count >= 128 branch
      "rle_1_127:            \n"
      "s8i  a8, a6, 0        \n" // *rle_buff_p = rle_val;
      //"movi a7, 0xFF         \n" // Not needed but reduce clock on ALLOW_ZERO_RLE op 1 27219->26428
      "s8i  a7, a6, 1        \n" // *rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
      "bgeui a5, 128, rle_1_127 \n" // if count >= 128 branch

      "rle_1_add:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // *
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "beqz a5, rle_1_skip   \n" 
      "or   a5, a5, a10      \n" // *
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p --
      "rle_1_skip:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1      

      "rle_1_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start:          \n"
      "l8ui a9, a4, 0        \n" // a9 as rle_val #0 is second
      "beq  a9, a8, rle_2_end\n" // rle_val == new_val continue

      "bltui a5, 128, rle_2_add \n" // if count >= 128 branch
      "rle_2_127:            \n"
      "s8i  a8, a6, 0        \n" // *rle_buff_p = rle_val;
      //"movi a7, 0xFF         \n" // Not needed but reduce clock on ALLOW_ZERO_RLE op 2
      "s8i  a7, a6, 1        \n" // *rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
      "bgeui a5, 128, rle_2_127 \n" // if count >= 128 branch

      "rle_2_add:            \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // *
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "beqz a5, rle_2_skip   \n"
      "or   a5, a5, a10      \n" // *
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p -- 
      "rle_2_skip:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p -- 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end:              \n"


      "bltui a5, 128, rle_end_add \n" // if count >= 128 branch
      "rle_end_127:          \n"
      "s8i  a8, a6, 0        \n" // *rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // *rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
 "bgeui a5, 128, rle_end_127 \n" // if count >= 128 branch

      "rle_end_add:          \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // *
      "s8i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
    "beqz a5, rle_end_skip   \n"
      "or   a5, a5, a10      \n" // *
      "s8i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p -- 
      "rle_end_skip:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p -- 
#endif

      "exit:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

      
    clockb = xthal_get_ccount();
    time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;
 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    

/*
    Serial_Debug_Port.printf("RLE Block Output:\r\n");
    for(int i=0; i < sample_size/40 ; i++ )
      Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
    Serial_Debug_Port.println();
    */
}

inline void fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU 
    * for capture 20Mhz sampling speed.
    */
/*
   Serial_Debug_Port.printf("RAW Block Input:\r\n");
   for(int i = 0; i < sample_size/40 ; i+=4 ){
     Serial_Debug_Port.printf("0x%X, ", dma_buffer[i+2]);
     Serial_Debug_Port.printf("0x%X, ", dma_buffer[i]);
     }
   Serial_Debug_Port.println();
*/
   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */
    
   int dword_count=(sample_size/4) -1;
   
   clocka = xthal_get_ccount();
   
   /* No, Assembly is not that hard. You are just too lazzy. */
   
    __asm__ __volatile__(
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
      "l32i a6, %2, 0        \n" // Load store rle_buffer address

      "l16ui a8, a4, 2       \n" // a8 as rle_val #2 is first
      "l16ui a9, a4, 0       \n" // a9 as new_val
      "beq  a9, a8, rle_0_16 \n" // rle_val == new_val skip

      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      //"movi a5, 0            \n" //                        *
      "s16i  a5, a6, 2        \n" // *rle_buff_p+2 = rle_counter; *
      "movi a5, -1           \n" // rle_counter=-1         *
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      //"addi a6, a6, 1        \n" // rle_buff_p ++
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0:                \n"
      "addi a5, a5, 1        \n" // rle_counter++
   "loopnez %1, rle_loop_end_16 \n" // Prepare zero-overhead loop
      "loopStart_16:         \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start16:                \n"
      "l16ui a9, a4, 2        \n" // a9 as rle_val #2 is first
   "beq  a9, a8, rle_1_end_16\n" // rle_val == new_val skip

      "rle_1_add_16:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "beqz a5, rle_1_skip_16   \n" 
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p --
      "rle_1_skip_16:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1      

      "rle_1_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_16:          \n"
      "l16ui a9, a4, 0        \n" // a9 as rle_val #0 is second
      "beq  a9, a8, rle_2_end_16\n" // rle_val == new_val continue

      "rle_2_add:            \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "beqz a5, rle_2_skip_16 \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p -- 
      "rle_2_skip_16:        \n"
      "addi a6, a6, 1        \n" // rle_buff_p -- 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_16:      \n"

      "rle_end_add_16:       \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 1        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0        \n" // *rle_buff_p=rle_val;
    "beqz a5, rle_end_skip_16   \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2        \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p -- 
      "rle_end_skip_16:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p -- 
#endif

      "exit_16:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");
    clockb = xthal_get_ccount();
//    delay(10);

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
