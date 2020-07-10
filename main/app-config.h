/// CHANGE CONFIGURATION HERE!!
#undef SUMP_OVER_UART
#undef SUMP_OVER_UART 
#define  SCPI_ON_NETWORK 1
//#define  SUMP_ON_NETWORK 1
#undef  DEBUG_SUMP 
//#define DEBUG_LOG_ENABLE 1
//#define OTA_RUN_SERVER 1

// First pin for parallell input 13-20
#define PARALLEL_0  12

#define UART_TEST_OUTPUT 1
#define UART_OUTPUT_PIN 17
#define UART_RX_PIN 19

// This might be used for sampling with DMA
//#define USE_CAMERA_IF 1
#define PIXEL_LEDC_PIN 33

#define PULSE_PIN 14
#define RMT_PULSES 1


#undef RUN_IN_QEMU

