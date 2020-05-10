#if 1
#include "analog.h"
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <esp_err.h>
//#include "esp_adc_cal.h"
#include "app-config.h"
#include "sdkconfig.h"
#define USE_SEMA 1
#include "soc/efuse_reg.h"


#ifdef CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ
#define CPU_FREQ CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ
#else
#define CPU_FREQ CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#endif

int trig_pin=-1;

//At initialization, you need to configure all 8 pins a GPIOs, e.g. by setting them all as inputs:

void setup_digital() {
  for (int i = 0; i < 16; i++) {

    if ((PARALLEL_0 +i == UART_OUTPUT_PIN)  || (PARALLEL_0 +i == UART_RX_PIN)) 
    {
        // If uart output is enabled, we do not set those pins at input
#ifndef UART_TEST_OUTPUT 
        gpio_set_direction(PARALLEL_0 +i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(PARALLEL_0 +i,GPIO_FLOATING);
#else
        printf("No input on pin %d because of uart testsignal D%d!\n",PARALLEL_0 +i,i);
#endif
    } else if (PARALLEL_0 +i == PULSE_PIN) 
    {
#ifndef RMT_PULSES
        gpio_set_direction(PARALLEL_0 +i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(PARALLEL_0 +i,GPIO_FLOATING);
#else
        printf("No input on pin %d because of rmt testsignal D%d!\n",PARALLEL_0 +i,i);
#endif

     }
#if 0
     else if ((PARALLEL_0 +i)==PIXEL_LEDC_PIN)
     {
        printf("No input because of pixelclock on pin %d D%d!\n",PARALLEL_0 +i,i);
     }
#endif     
     else {
        gpio_set_direction( PARALLEL_0 + GPIO_NUM_0 +i,GPIO_MODE_INPUT);
        gpio_set_pull_mode( PARALLEL_0 + GPIO_NUM_0 +i,GPIO_FLOATING);
    }
  }
}

//After that, you can use the following functions to set the 8 pins as inputs or outputs, and to read the input values and write the output values:
void parallel_set_inputs(void) {
  REG_WRITE(GPIO_ENABLE_W1TC_REG, 0xFF << PARALLEL_0);
}

void parallel_set_outputs(void) {
  REG_WRITE(GPIO_ENABLE_W1TS_REG, 0xFF << PARALLEL_0);
}

inline uint16_t parallel_read(void) {
  uint32_t input = REG_READ(GPIO_IN_REG);
  uint16_t ret=input >> PARALLEL_0;

  return (ret);
}

void parallel_write(uint8_t value) {
  uint32_t output =
    (REG_READ(GPIO_OUT_REG) & ~(0xFF << PARALLEL_0)) | (((uint32_t)value) << PARALLEL_0);

  REG_WRITE(GPIO_OUT_REG, output);
}

/*Note: Different ESP32 modules may have different reference voltages varying from
 * 1000mV to 1200mV. Use #define GET_VREF to route v_ref to a GPIO
 */
#define V_REF   1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_0)
      //GPIO 36

uint8_t analouge_values[NUM_SAMPLES];

int analouge_in_values[NUM_SAMPLES];

uint16_t digital_in_values[NUM_SAMPLES];

// Every 20 th value is the clock count
uint16_t cc_and_digital[NUM_SAMPLES*20];


int sample_point;


SemaphoreHandle_t xSemaphore = NULL;

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// TODO, use DMA  , adc_set_i2s_data_source, 
// Also allow setting parameters from sigrok

// A complete sample loop takes about 8000 cycles, will not go faster
#define COUNT_FOR_SAMPLE CPU_FREQ * 10000*0.01

int ccount_delay=COUNT_FOR_SAMPLE;

void setTimescale(float scale){

  ccount_delay= CPU_FREQ *10000*scale;  // 160
  printf("ccount_delay=%d\n",ccount_delay);

}




// This function can be used to find the true value for V_REF
void route_adc_ref()
{
    esp_err_t status = adc2_vref_to_gpio(GPIO_NUM_25);
    if (status == ESP_OK){
        printf("v_ref routed to GPIO\n");
    }else{
        printf("failed to route v_ref\n");
    }
    fflush(stdout);
}

//-----------------------------------------------------------------------------
// Read CCOUNT register.
//-----------------------------------------------------------------------------
// inline
static  uint32_t get_ccount(void)
{
  uint32_t ccount;

  asm volatile ( "rsr     %0, ccount" : "=a" (ccount) );
  return ccount;
}


uint32_t last_ccount=0;
// inline
static  uint32_t get_delta(void) {
    uint32_t ret;
    uint32_t new_ccount=get_ccount();
    if (last_ccount<new_ccount) {
        ret=new_ccount-last_ccount;
       last_ccount=new_ccount;
       return(ret);
    } else {
        ret=0xffffffff-last_ccount + new_ccount;
        last_ccount=new_ccount;
        return (ret);
    }

}

/*
NOTE: When read as unsigned decimals, each byte should have a value of between 15 and 240. 
The top of the LCD display of the scope represents byte value 25 and the bottom is 225. 

• <Volts_Div>  : Returned time/div value from scope
• <Raw_Byte>: Raw byte from array
• <Vert_Offset>  : Returned Vertical offset value from scope

For each point, to get the amplitude (A) of the waveform in volts (V):

A(V) = [(240 -<Raw_Byte> ) * (<Volts_Div> / 25) - [(<Vert_Offset> + <Volts_Div> * 4.6)]] 

We use vert_offset=0
Volts_Div=4 i think.

A(V) = [(240 -<Raw_Byte> ) * (<Volts_Div> / 25) - (<Volts_Div> * 4.6)] 

A(V) +  (<VD> * 4.6) = [(240 -<Raw_Byte> ) * (<VD> / 25) ] 

AV/VD + 4.6/VD = (240 -R) / 25

(AV/VD + 4.6/VD ) *25 = 240 - R 

<Raw_Byte> = 240 -  25*( A(V)/VD + 4.6/VD) 

*/
#define VD 0.5

uint8_t voltage_to_RawByte(uint32_t voltage) {
    uint8_t ret=240 - 25*(voltage/(1000.0*VD) +4.6/VD) ;

    return(ret);
}

int* get_sample_values() {
    /*
    Use this for calibrated values

    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);

    TODO,
    Use esp_adc_cal_characterize()

    */

    sample_point=0;
    for(int i=0;i<NUM_SAMPLES;i++) {
      uint32_t mv= analouge_in_values[sample_point];  //esp_adc_cal_raw_to_voltage(analouge_in_values[sample_point]	, &characteristics);
        analouge_in_values[sample_point]=mv;
        sample_point++;
    }

    sample_point=0;
    return analouge_in_values;
}


uint8_t* get_values() {
  /*
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);

    sample_point=0;
    for(int i=0;i<NUM_SAMPLES;i++) {
        uint32_t mv=esp_adc_cal_raw_to_voltage(analouge_in_values[sample_point], &characteristics);
        analouge_values[sample_point]=voltage_to_RawByte(mv);
        sample_point++;
    }
  */
    sample_point=0;
    return analouge_values;
};

uint16_t* get_digital_values() {
    if (ccount_delay<8000) { 
        printf("RESAMP\n");
#if 0
        int sampleIx=0;
        for (int sample_point=0;sample_point<NUM_SAMPLES;sample_point++) {
            if (sample_point%20!=0) {
                digital_in_values[sampleIx++]=cc_and_digital[sample_point];
            } 
        }
    }
#endif
        // Resample to desired rate
        int sampleIx=0;
        uint32_t oneSample_time=0;
        int delta=0;
        // First time around we get a cache miss, then delays becomes stable
        for (int sample_point=20;sample_point<(NUM_SAMPLES*19) && (sampleIx<NUM_SAMPLES);sample_point+=20) {
            oneSample_time=cc_and_digital[sample_point+20]/20;
            for (int j=1;j<20;j++) {
                delta+=oneSample_time;
                if(delta>ccount_delay) {
                       digital_in_values[sampleIx]=cc_and_digital[sample_point+j];
                       //printf(" %d-",j); 
                       sampleIx++;
                       delta-=ccount_delay;
                }
            }       
            //printf(" %d %d\n",sampleIx,cc_and_digital[sample_point]);
        }
    }
   return digital_in_values;
}

static int maxSamples=NUM_SAMPLES;

void set_mem_depth(int depth) {
    if (depth<NUM_SAMPLES) {
        maxSamples=depth;
    }
}

bool stop_aq=false;
void stop_aquisition() {
    stop_aq=true;
}


//int time_called=0;;
void sample_thread(void *param) {

    stop_aq=false;
    TaskHandle_t *notify_task=(TaskHandle_t *)param;

    for (int i=0;i<NUM_SAMPLES;i++) {
        digital_in_values[i]=0;
    }

    //Init ADC and Characteristics
    /*
    esp_adc_cal_characteristics_t characteristics;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);
    */
#if 0
    uint32_t voltage;
    while(1){
        voltage = adc1_to_voltage(ADC1_TEST_CHANNEL, &characteristics);
        printf("%d mV\n",voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    uint32_t voltage;

    uint32_t ccount;

    int accumulated_ccount=0;
    // Initate
    uint32_t test=get_delta();
    //printf("%d\n",test);
    //test=get_delta();
    //printf("%d\n",test);
    test=get_delta();
    //printf("%d\n",test);
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10 );
    bool got_sem=false;

#if USE_SEMA
    if (xSemaphoreTake(xSemaphore,xMaxBlockTime)) {
       got_sem=true;
    }
#endif

#define DUMMY EFUSE_BLK0_RDATA3_REG

    sample_point=0;

    test=get_delta();

    int breakout=0;
    if (trig_pin>=0) {
        printf("Waiting for trigger %d\n",trig_pin);
        uint16_t tmp=parallel_read();
        tmp=parallel_read();
        uint16_t next=parallel_read();

        while ((((1<<trig_pin) & tmp)==((1<<trig_pin) & next)) && (stop_aq==false)) {
            uint32_t dummy = REG_READ(DUMMY);
            // Check if value changed
            next=parallel_read();
             if (breakout++>10000) {
                 next=!tmp;
             }
        }
        printf("Trigged at %d %X %X\n",breakout,tmp,next);
    }



    test=get_delta();

    if (ccount_delay<8000) { 
        printf("-----------------------\n");

        // Disable  C callable interrupts 
        //portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
        //taskENTER_CRITICAL(&myMutex);
        while (sample_point<NUM_SAMPLES*19) {

            // Every 20 th value is the lower 16 bits of the clock count
            test=get_delta();
            cc_and_digital[sample_point]= test & 0xffff;
            //digital_in_values[sample_point+1]= test >> 16;
//GPIO_STRAP_REG
            // The dummy reads are to create a delay and for workaround
            uint32_t dummy = REG_READ(DUMMY);
            cc_and_digital[sample_point+1]=parallel_read();
            dummy = REG_READ(DUMMY);
            cc_and_digital[sample_point+2]=parallel_read();
            dummy = REG_READ(DUMMY);
            cc_and_digital[sample_point+3]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+4]=parallel_read();
            dummy = REG_READ(DUMMY);                   
            cc_and_digital[sample_point+5]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+6]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+7]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+8]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+9]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+10]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+11]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+12]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+13]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+14]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+15]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+16]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+17]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+18]=parallel_read();
            dummy = REG_READ(DUMMY);          
            cc_and_digital[sample_point+19]=parallel_read();

            // TDOD, RAW analouge values, this confuses timing
            //voltage = adc1_get_raw(ADC1_TEST_CHANNEL);
            //analouge_in_values[sample_point/20]=voltage;
            //taskYIELD();

            sample_point+=20;
        }
        //end critical section
        //taskEXIT_CRITICAL(&myMutex);

    } else {
 
         printf("ooooooooooooooooo\n");

        sample_point=0;
        // cache
        //digital_in_values[sample_point]=parallel_read();
        //digital_in_values[NUM_SAMPLES/2]=parallel_read();

        while (sample_point<maxSamples && (stop_aq==false)) {
            while (accumulated_ccount<ccount_delay) {
                taskYIELD();
                accumulated_ccount+=get_delta();
                if (stop_aq==true) {
                    break;
                }
            }
            // Max digital..
            voltage = adc1_get_raw(ADC1_TEST_CHANNEL);

            //voltage = adc1_to_voltage(ADC1_TEST_CHANNEL, &characteristics);

            digital_in_values[sample_point]=parallel_read();
            //sample_point++;
            analouge_in_values[sample_point++]=voltage;
            //if (time_called++%100==0) {
            //    printf("-%d\n",accumulated_ccount);    
            //}

            accumulated_ccount-=ccount_delay;
        }
    }

    printf("++++++++++++++++++++\n");


#if USE_SEMA
    if (got_sem) {
       xSemaphoreGive(xSemaphore);
    }
    vTaskDelete(NULL);
#endif
return;
}

//  xTaskCreatePinnedToCore(&sample_thread, "sample_thread", 4096, NULL, 20, NULL, 1);



void start_sampling() {

    if (xSemaphore==NULL) {
        setup_digital();
        xSemaphore = xSemaphoreCreateMutex();
    }

#if USE_SEMA
    xTaskCreatePinnedToCore(&sample_thread, "sample_thread", 4096, NULL, 20, &xHandlingTask, 1);
#else
    sample_thread(NULL);
#endif
}

bool samples_finnished() {
    bool ret=false;

#if USE_SEMA

    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 100000 );

    if (xSemaphoreTake(xSemaphore,xMaxBlockTime)) {
        ret=true;
        xSemaphoreGive(xSemaphore);
    }
#else
        ret=true;
#endif

    return ret;
}
#endif
