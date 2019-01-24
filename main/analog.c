#include "analog.h"
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include "esp_adc_cal.h"

#define USE_SEMA 0

/*Note: Different ESP32 modules may have different reference voltages varying from
 * 1000mV to 1200mV. Use #define GET_VREF to route v_ref to a GPIO
 */
#define V_REF   1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_0)
      //GPIO 36

uint8_t analouge_values[NUM_SAMPLES];

int analouge_in_values[NUM_SAMPLES];

int sample_point;


SemaphoreHandle_t xSemaphore = NULL;


// TODO, use DMA  , adc_set_i2s_data_source, 
// Also allow setting parameters from sigrok

// A complete sample loop takes about 8000 cycles, will not go faster
#define COUNT_FOR_SAMPLE 17000


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
    */
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);

    sample_point=0;
    for(int i=0;i<NUM_SAMPLES;i++) {
        uint32_t mv=esp_adc_cal_raw_to_voltage(analouge_in_values[sample_point], &characteristics);
        analouge_in_values[sample_point]=mv;
        sample_point++;
    }

    sample_point=0;
    return analouge_in_values;
}


uint8_t* get_values() {
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);

    sample_point=0;
    for(int i=0;i<NUM_SAMPLES;i++) {
        uint32_t mv=esp_adc_cal_raw_to_voltage(analouge_in_values[sample_point], &characteristics);
        analouge_values[sample_point]=voltage_to_RawByte(mv);
        sample_point++;
    }

    sample_point=0;
    return analouge_values;
};



void sample_thread(void *param) {

    TaskHandle_t *notify_task=(TaskHandle_t *)param;

    //Init ADC and Characteristics
    esp_adc_cal_characteristics_t characteristics;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);

#if 0
    uint32_t voltage;
    while(1){
        voltage = adc1_to_voltage(ADC1_TEST_CHANNEL, &characteristics);
        printf("%d mV\n",voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    uint32_t voltage;
    int blink=0;

    uint32_t ccount;

    uint32_t accumulated_ccount=0;
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

    sample_point=0;
    while (sample_point<NUM_SAMPLES) {
        while (accumulated_ccount<COUNT_FOR_SAMPLE) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            accumulated_ccount+=get_delta();
        }
        voltage = adc1_get_raw(ADC1_TEST_CHANNEL);
        //voltage = adc1_to_voltage(ADC1_TEST_CHANNEL, &characteristics);
        // gpio_set_level(GPIO_NUM_17, blink);
        analouge_in_values[sample_point++]=voltage;
        blink=!blink;
        accumulated_ccount-=COUNT_FOR_SAMPLE;
    }

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
