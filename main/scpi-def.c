/*-
 * Copyright (c) 2012-2013 Jan Breuer,
 *
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi-def.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  SCPI parser test
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

#define START_SAMPLE_TASK 1

int32_t memLen=1400;


#ifdef START_SAMPLE_TASK
#include "analog.h"
#endif

int wav_read_position=0;

extern int trig_pin;





static scpi_result_t measure_count_source(scpi_t * context) {
    fprintf(stderr, "meas:count:source\r\n"); /* debug command name */

    SCPI_ResultMnemonic(context, "OFF");
    return SCPI_RES_OK;

}

static scpi_result_t DMM_MeasureVoltageDcQ(scpi_t * context) {
    scpi_number_t param1, param2;
    char bf[15];
    fprintf(stderr, "meas:volt:dc\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
        /* do something, if parameter not present */
    }

    /* read second paraeter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
        /* do something, if parameter not present */
    }


    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
    fprintf(stderr, "\tP1=%s\r\n", bf);


    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);
    fprintf(stderr, "\tP2=%s\r\n", bf);

    SCPI_ResultDouble(context, 0);

    return SCPI_RES_OK;
}

static scpi_result_t DMM_MeasureVoltageAcQ(scpi_t * context) {
    scpi_number_t param1, param2;
    char bf[15];
    fprintf(stderr, "meas:volt:ac\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
        /* do something, if parameter not present */
    }

    /* read second paraeter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
        /* do something, if parameter not present */
    }


    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
    fprintf(stderr, "\tP1=%s\r\n", bf);


    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);
    fprintf(stderr, "\tP2=%s\r\n", bf);

    SCPI_ResultDouble(context, 0);

    return SCPI_RES_OK;
}

static scpi_result_t DMM_ConfigureVoltageDc(scpi_t * context) {
    double param1, param2;
    fprintf(stderr, "conf:volt:dc\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    /* read second paraeter if present */
    if (!SCPI_ParamDouble(context, &param2, FALSE)) {
        /* do something, if parameter not present */
    }

    fprintf(stderr, "\tP1=%lf\r\n", param1);
    fprintf(stderr, "\tP2=%lf\r\n", param2);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Bool(scpi_t * context) {
    scpi_bool_t param1;
    fprintf(stderr, "TEST:BOOL\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamBool(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    fprintf(stderr, "\tP1=%d\r\n", param1);

    return SCPI_RES_OK;
}

scpi_choice_def_t trigger_source[] = {
    {"BUS", 5},
    {"IMMediate", 6},
    {"EXTernal", 7},
    SCPI_CHOICE_LIST_END /* termination of option list */
};

static scpi_result_t TEST_ChoiceQ(scpi_t * context) {

    int32_t param;
    const char * name;

    if (!SCPI_ParamChoice(context, trigger_source, &param, TRUE)) {
        return SCPI_RES_ERR;
    }

    SCPI_ChoiceToName(trigger_source, param, &name);
    fprintf(stderr, "\tP1=%s (%ld)\r\n", name, (long int) param);

    SCPI_ResultInt32(context, param);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Numbers(scpi_t * context) {
    int32_t numbers[2];

    SCPI_CommandNumbers(context, numbers, 2, 1);

    fprintf(stderr, "TEST numbers %d %d\r\n", numbers[0], numbers[1]);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Text(scpi_t * context) {
    char buffer[100];
    size_t copy_len;

    if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, FALSE)) {
        buffer[0] = '\0';
    }

    fprintf(stderr, "TEXT: ***%s***\r\n", buffer);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_ArbQ(scpi_t * context) {
    const char * data;
    size_t len;

    if (SCPI_ParamArbitraryBlock(context, &data, &len, FALSE)) {
        SCPI_ResultArbitraryBlock(context, data, len);
    }

    return SCPI_RES_OK;
}

struct _scpi_channel_value_t {
    int32_t row;
    int32_t col;
};
typedef struct _scpi_channel_value_t scpi_channel_value_t;

/**
 * @brief
 * parses lists
 * channel numbers > 0.
 * no checks yet.
 * valid: (@1), (@3!1:1!3), ...
 * (@1!1:3!2) would be 1!1, 1!2, 2!1, 2!2, 3!1, 3!2.
 * (@3!1:1!3) would be 3!1, 3!2, 3!3, 2!1, 2!2, 2!3, ... 1!3.
 *
 * @param channel_list channel list, compare to SCPI99 Vol 1 Ch. 8.3.2
 */
static scpi_result_t TEST_Chanlst(scpi_t *context) {
    scpi_parameter_t channel_list_param;
#define MAXROW 2    /* maximum number of rows */
#define MAXCOL 6    /* maximum number of columns */
#define MAXDIM 2    /* maximum number of dimensions */
    scpi_channel_value_t array[MAXROW * MAXCOL]; /* array which holds values in order (2D) */
    size_t chanlst_idx; /* index for channel list */
    size_t arr_idx = 0; /* index for array */
    size_t n, m = 1; /* counters for row (n) and columns (m) */

    /* get channel list */
    if (SCPI_Parameter(context, &channel_list_param, TRUE)) {
        scpi_expr_result_t res;
        scpi_bool_t is_range;
        int32_t values_from[MAXDIM];
        int32_t values_to[MAXDIM];
        size_t dimensions;

        bool for_stop_row = FALSE; /* true if iteration for rows has to stop */
        bool for_stop_col = FALSE; /* true if iteration for columns has to stop */
        int32_t dir_row = 1; /* direction of counter for rows, +/-1 */
        int32_t dir_col = 1; /* direction of counter for columns, +/-1 */

        /* the next statement is valid usage and it gets only real number of dimensions for the first item (index 0) */
        if (!SCPI_ExprChannelListEntry(context, &channel_list_param, 0, &is_range, NULL, NULL, 0, &dimensions)) {
            chanlst_idx = 0; /* call first index */
            arr_idx = 0; /* set arr_idx to 0 */
            do { /* if valid, iterate over channel_list_param index while res == valid (do-while cause we have to do it once) */
                res = SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions);
                if (is_range == FALSE) { /* still can have multiple dimensions */
                    if (dimensions == 1) {
                        /* here we have our values
                         * row == values_from[0]
                         * col == 0 (fixed number)
                         * call a function or something */
                        array[arr_idx].row = values_from[0];
                        array[arr_idx].col = 0;
                    } else if (dimensions == 2) {
                        /* here we have our values
                         * row == values_fom[0]
                         * col == values_from[1]
                         * call a function or something */
                        array[arr_idx].row = values_from[0];
                        array[arr_idx].col = values_from[1];
                    } else {
                        return SCPI_RES_ERR;
                    }
                    arr_idx++; /* inkrement array where we want to save our values to, not neccessary otherwise */
                    if (arr_idx >= MAXROW * MAXCOL) {
                        return SCPI_RES_ERR;
                    }
                } else if (is_range == TRUE) {
                    if (values_from[0] > values_to[0]) {
                        dir_row = -1; /* we have to decrement from values_from */
                    } else { /* if (values_from[0] < values_to[0]) */
                        dir_row = +1; /* default, we increment from values_from */
                    }

                    /* iterating over rows, do it once -> set for_stop_row = false
                     * needed if there is channel list index isn't at end yet */
                    for_stop_row = FALSE;
                    for (n = values_from[0]; for_stop_row == FALSE; n += dir_row) {
                        /* usual case for ranges, 2 dimensions */
                        if (dimensions == 2) {
                            if (values_from[1] > values_to[1]) {
                                dir_col = -1;
                            } else if (values_from[1] < values_to[1]) {
                                dir_col = +1;
                            }
                            /* iterating over columns, do it at least once -> set for_stop_col = false
                             * needed if there is channel list index isn't at end yet */
                            for_stop_col = FALSE;
                            for (m = values_from[1]; for_stop_col == FALSE; m += dir_col) {
                                /* here we have our values
                                 * row == n
                                 * col == m
                                 * call a function or something */
                                array[arr_idx].row = n;
                                array[arr_idx].col = m;
                                arr_idx++;
                                if (arr_idx >= MAXROW * MAXCOL) {
                                    return SCPI_RES_ERR;
                                }
                                if (m == (size_t)values_to[1]) {
                                    /* endpoint reached, stop column for-loop */
                                    for_stop_col = TRUE;
                                }
                            }
                            /* special case for range, example: (@2!1) */
                        } else if (dimensions == 1) {
                            /* here we have values
                             * row == n
                             * col == 0 (fixed number)
                             * call function or sth. */
                            array[arr_idx].row = n;
                            array[arr_idx].col = 0;
                            arr_idx++;
                            if (arr_idx >= MAXROW * MAXCOL) {
                                return SCPI_RES_ERR;
                            }
                        }
                        if (n == (size_t)values_to[0]) {
                            /* endpoint reached, stop row for-loop */
                            for_stop_row = TRUE;
                        }
                    }


                } else {
                    return SCPI_RES_ERR;
                }
                /* increase index */
                chanlst_idx++;
            } while (SCPI_EXPR_OK == SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions));
            /* while checks, whether incremented index is valid */
        }
        /* do something at the end if needed */
        /* array[arr_idx].row = 0; */
        /* array[arr_idx].col = 0; */
    }

    {
        size_t i;
        fprintf(stderr, "TEST_Chanlst: ");
        for (i = 0; i< arr_idx; i++) {
            fprintf(stderr, "%d!%d, ", array[i].row, array[i].col);
        }
        fprintf(stderr, "\r\n");
    }
    return SCPI_RES_OK;
}

/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t * context) {

    SCPI_ResultInt32(context, 0);

    return SCPI_RES_OK;
}

/**
 * OLAS test
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t chan_disp_on(scpi_t * context) {

    SCPI_ResultMnemonic(context, "1");

    return SCPI_RES_OK;
}

static scpi_result_t query_disp_grid(scpi_t * context) {

    SCPI_ResultMnemonic(context, "FULL");

    return SCPI_RES_OK;
}

static scpi_result_t query_disp_grad_time(scpi_t * context) {

    SCPI_ResultMnemonic(context, "MIN");

    return SCPI_RES_OK;
}



// VECT or DOT
static scpi_result_t query_disp_type(scpi_t * context) {

    SCPI_ResultMnemonic(context, "VECT");

    return SCPI_RES_OK;
}





static scpi_result_t chan_disp_off(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");
    //SCPI_ResultBool(context, 0);

    return SCPI_RES_OK;
}
/*
24
Example:
:TIMebase:MODE

MAIN vSet
the main timebase
:TIMebase:OFFSet

1vSet the timebase offset as 1s
:TIMebase:OFFSet? 
Query returns
1.000e+00

3.
:
TIMebase [:DELayed]:SCALe
Command Format: 
:TIMebase[:DELayed]:SCALe
 
<scale_val>
The command is to set 
the time base scale in 
“MAIN” or “DELAYE” mode
.
The unit  is  “S/div”
.
When the “Delayed” is “ON”, change the Delayed Timebase Scale will 
change the width of window to amplify waveform
.
<scale_val>is  2ns - 50s
when it is Normal mode
500ms -50s when it is Roll mode

When “MAIN” is “ON”, omit [: DELayed]
.
Query Format
:
:TIMebase[:DELayed]:SCALe?
Returned Format:
Query returns
the set value of < scale_val>
in scientific numeric notation.

Example:
:TIMebase:MODE

DELayed 
Set the 
“MAIN‟ timebase
:TIMebase:SCALe

2
Set the timebase scale as 2s
:TIMebase:SCALe?
Query returns
2.000e+00
*/

// 5ms start time
float time_scale_value=0.01;

static scpi_result_t time_scale(scpi_t * context) {

    SCPI_ResultFloat(context, time_scale_value);
    // The unit  is  “S/div”
    // 10 ms/div
    //SCPI_ResultMnemonic(context, "10e-3");

    // 1 us/div
    //SCPI_ResultMnemonic(context, "10e-6");


    //SCPI_ResultText(context, "1e-6");
    //SCPI_ResultFloat(context, 2.0f);
    //SCPI_ResultBool(context, 0);

    return SCPI_RES_OK;
}

static scpi_result_t chan1_probe(scpi_t * context) {

/*
Explanation:
The  command  is  to  set  the  attenuation  factor  of  the  probe
 
1X,  10X,  100X , or 1000X
to keep the  Measurement exact
The options of <n>
are 1 or 2.
*/
    //SCPI_ResultFloat(context, 2.0f);

    SCPI_ResultMnemonic(context, "1");
    //SCPI_ResultFloat(context, 2.0f);
    //SCPI_ResultBool(context, 0);

    return SCPI_RES_OK;
}


float ch1_scale_value;
float ch2_scale_value;

static scpi_result_t set_chan1_scal(scpi_t * context) {

/*
The command is to set the vertical range of the amplified waveform.
<range> is  
2mV~ 10V       Probe 1X   
20mV~100V      Probe 10X   
200mV~ 1000V   Probe 100X  
2V~10000V      Probe 1000X 
*/
    if (SCPI_ParamFloat(context, &ch1_scale_value,TRUE)) {

    }

    SCPI_ResultFloat(context, ch1_scale_value);


    return SCPI_RES_OK;
}



static scpi_result_t set_chan2_scal(scpi_t * context) {

/*
The command is to set the vertical range of the amplified waveform.
<range> is  
2mV~ 10V       Probe 1X   
20mV~100V      Probe 10X   
200mV~ 1000V   Probe 100X  
2V~10000V      Probe 1000X 
*/
    if (SCPI_ParamFloat(context, &ch2_scale_value,TRUE)) {

    }

    SCPI_ResultFloat(context, ch2_scale_value);


    return SCPI_RES_OK;
}


static scpi_result_t chan1_scal(scpi_t * context) {

/*
The command is to set the vertical range of the amplified waveform.
<range> is  
2mV~ 10V       Probe 1X   
20mV~100V      Probe 10X   
200mV~ 1000V   Probe 100X  
2V~10000V      Probe 1000X 
*/
    SCPI_ResultFloat(context, ch1_scale_value);


    return SCPI_RES_OK;
}

static scpi_result_t chan2_scal(scpi_t * context) {

    SCPI_ResultFloat(context, ch2_scale_value);

    return SCPI_RES_OK;
}


static scpi_result_t chan1_offset(scpi_t * context) {

/*
The command is to set the vertical
offset of the waveform
. 
When scale >100mV
, the range of  <Offset>
is -40V~ +40V
When Scale <=100mV
the range of <Offset>
is -2V ~ +2V
*/
    SCPI_ResultFloat(context, 0.0f);


    return SCPI_RES_OK;
}

static scpi_result_t chan1_coup(scpi_t * context) {

/*
Returned Format:
Query returns “DC”,“AC” or “GND”. 
The double quotes are not returned.
*/
    SCPI_ResultMnemonic(context, "DC");


    return SCPI_RES_OK;
}



static scpi_result_t trig_edge_source(scpi_t * context) {

/*
Query 
returns “CH1”, “CH2”, “EXT”, 
“EXT5”or “DIGITAL”
*/
    if (trig_pin==1) {
        SCPI_ResultMnemonic(context, "D1");
    } else {
        SCPI_ResultMnemonic(context, "D0");
    }

    return SCPI_RES_OK;
}


static scpi_result_t la_stat_query(scpi_t * context) {

/*
Returned Format:
Query returns “DC”,“AC” or “GND”. 
The double quotes are not returned.
*/
    SCPI_ResultMnemonic(context, "1");


    return SCPI_RES_OK;
}


static scpi_result_t tim_offset(scpi_t * context) {

/*
The command is to adjust 
the timebase offset in 
“MAIN” or
“Delayed”
mode.
<offset>
is 
1s ~ momery terminal
when it is Normal
mode
-
500s ~ +500s
when it is Stop
mode
-
6*
S
cale ~ +6*Scale
when  it  is  Roll  mode
(
“Scale”
indicates 
the current 
horizontal
scale
. 
T
he default 
unit is s/div
)
When it i
s “MAIN”
, 
omit [:DELayed]
.
Query Format
:
:TIMebase[:DELayed]:OFFSet?
*/
    SCPI_ResultFloat(context, 0.0);


    return SCPI_RES_OK;
}


static scpi_result_t trig_edge_slope(scpi_t * context) {

/* Query 
r
eturns “POSITIVE”
or 
“NEGATIVE”
. */
     SCPI_ResultMnemonic(context, "POSITIVE");


    return SCPI_RES_OK;
}


static scpi_result_t trig_edge_level(scpi_t * context) {
  
  /*   Edge  trigger level  */

   SCPI_ResultFloat(context, 2.0);

   return SCPI_RES_OK;
}

int times_called=0;


static scpi_result_t run_to_the_hills(scpi_t * context) {
  /*
  The  command  initiates
the  oscilloscope  to  acquire waveform  data  according  to  its 
current settings. Acquisition runs continuously until the oscilloscope receives a :STOP 
command,  or  a  single  acquisition  has  occurred  when  the  Trigger 
mode is  set  to  “Single”.
*/
times_called=0;
printf("RUN\n");
#ifdef START_SAMPLE_TASK
start_sampling(false);
#endif
return SCPI_RES_OK;
}

void stop_aquisition();

static scpi_result_t stop_acquisition(scpi_t * context) {
/*
The  command  controls  the  oscilloscope  to  stop  acquiring  data.  To  restart  the 
acquisition, use the :RUN command
*/
printf("stop-\n");
stop_aquisition();
return SCPI_RES_OK;
}

static scpi_result_t query_trig_coup(scpi_t * context) {
    SCPI_ResultMnemonic(context, "DC");
    return SCPI_RES_OK;
}

typedef enum {
    trig_normal,
    trig_auto,
    trig_single    
} t_TrigSweep;

t_TrigSweep sweep=trig_auto;

static scpi_result_t trig_swe(scpi_t * context) {
   char *result;
    size_t len;
 
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        printf("SET_SWEEP %s\n",result);

        if (strncmp(result,"NORM",4)==0) 
        {
            printf("NORM\n");
            sweep=trig_normal;
        } else  if (strncmp(result,"AUTO",4)==0) {
            printf("AUTO\n");
            sweep=trig_auto;
        } else  if (strncmp(result,"SING",4)==0) {
            printf("SING\n");
            sweep=trig_single;
        }
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}


static scpi_result_t query_trig_sweep(scpi_t * context) {
    switch (sweep) {
        case trig_normal:
            SCPI_ResultMnemonic(context, "NORM");
        break;
        case trig_auto:
            SCPI_ResultMnemonic(context, "AUTO");
        break;
        case trig_single:
            SCPI_ResultMnemonic(context, "SING");
        break;
    }
    return SCPI_RES_OK;
}






static scpi_result_t query_trig_mode(scpi_t * context) {
    SCPI_ResultMnemonic(context, "EDGE");
    return SCPI_RES_OK;
}

static scpi_result_t query_trig_slope(scpi_t * context) {
    SCPI_ResultMnemonic(context, "POS");
    return SCPI_RES_OK;
}

TrigType_t gSlopeTrigType=Pos;

static scpi_result_t query_trig_edg_slope(scpi_t * context) {
    SCPI_ResultMnemonic(context, "POS");
    return SCPI_RES_OK;
}

static scpi_result_t set_trig_edg_slope(scpi_t * context) {
    setAnalogTrig(Pos);
    return SCPI_RES_OK;
}




static scpi_result_t query_trig_edg_source(scpi_t * context) {
    SCPI_ResultMnemonic(context, "CHAN1");
    return SCPI_RES_OK;
}


// Set source
static scpi_result_t trig_edg_source(scpi_t * context) {

    char *result;
    size_t len;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        printf("Trig edge %s\n",result);
        return SCPI_RES_OK;
    }



    return SCPI_RES_OK;
}




static scpi_result_t trig_status(scpi_t * context) {

/*
* Trigger status may return:
* "TD" or "T'D" - triggered
* "AUTO"        - autotriggered
* "RUN"         - running
* "WAIT"        - waiting for trigger
* "STOP"        - stopped

*/
#if START_SAMPLE_TASK

TrigState_t trig_state=get_trig_state();

   switch(trig_state) {
       case Triggered:
         SCPI_ResultMnemonic(context, "TD");
         printf("TD");
       break;
       case Auto:
          SCPI_ResultMnemonic(context, "AUTO");
          printf("AUTO");
       break;
       case Running:
          SCPI_ResultMnemonic(context, "RUN");
          printf("RUN");          
       break;
       case Waiting:
          SCPI_ResultMnemonic(context, "WAIT");
          printf("WAIT");          
       break;
       case Stopped:
          SCPI_ResultMnemonic(context, "STOP");
         printf("STOP");
       break;
       default:
          SCPI_ResultMnemonic(context, "STOP");
       break;
   }

#else
times_called++;

          SCPI_ResultMnemonic(context, "AUTO");
return SCPI_RES_OK;


if (times_called>10) {
  SCPI_ResultMnemonic(context, "STOP");
  printf("STOP-");
} else if (times_called>4) {
  SCPI_ResultMnemonic(context, "TD");
  printf("TD-");
} else if (times_called>2) {
 SCPI_ResultMnemonic(context, "RUN");
  printf("RUN-");
} else {
  printf("WAIT-");
 SCPI_ResultMnemonic(context, "WAIT");    
}
#endif

return SCPI_RES_OK;
}

unsigned char sample_data[14000+12];

bool readDigital=false;



//http://int.rigol.com/File/TechDoc/20151218/MSO1000Z&DS1000Z_ProgrammingGuide_EN.pdf
static scpi_result_t wav_data(scpi_t * context) {
    const char * data;
    //size_t len=1400;

    //printf("wav_data ");
#ifdef START_SAMPLE_TASK
    samples_finnished();
#endif

    // SCPI_ResultArbitraryBlock(,1400) will add this header
    // #90 0000 1400 1400


#ifdef START_SAMPLE_TASK
//

if (readDigital) {
    printf("DIG\n");
    uint8_t* result=(uint8_t *)get_digital_values();
    for (int i=0;i<memLen;i++) {
        sample_data[i]=*result;
        result++;
    }
} else {
    printf("ANAL\n");

    uint8_t* result=get_values();
    for (int i=0;i<memLen;i++) {
        sample_data[i]=*result;
        result++;
    }

}

#else

  if (readDigital) {
    int fake=0xaa;
    for (int i=0;i<14000;i+=1) {
        //sprintf("%2X",&sample_data[i*2],(int)3.0*i/2048);
       sample_data[i]=fake;
       if ((i/100%2)==0) {
        fake=0x55;
       } else {
           fake=0xaa;
       }
       //if (fake>225) {
       //    fake=25;
       // }
    }
  } else {
    int fake=0x0;
    for (int i=11;i<memLen;i+=1) {
        //sprintf("%2X",&sample_data[i*2],(int)3.0*i/2048);
       sample_data[i]=fake++;
       if (fake>225) {
           fake=0;
        }
    }

  }
#endif
    SCPI_ResultArbitraryBlock(context,sample_data,memLen);

   return SCPI_RES_OK;
}

static scpi_result_t wav_yref(scpi_t * context) {

    SCPI_ResultMnemonic(context, "127");

    return SCPI_RES_OK;
}


static scpi_result_t wav_stat(scpi_t * context) {
    char tmp_buff[20];

    if (get_trig_state()==Running) {
      sprintf(tmp_buff,"READ,%d",memLen);
      stop_aquisition();
      printf("---->%s",tmp_buff);
    } else {
      sprintf(tmp_buff,"IDLE,%d",memLen);
      stop_aquisition();
      printf("---->%s",tmp_buff);
    }
    SCPI_ResultCharacters(context, tmp_buff,strlen(tmp_buff));

    return SCPI_RES_OK;
}

static scpi_result_t query_wav_yinc(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0.001e+00");

    return SCPI_RES_OK;
}

static scpi_result_t query_bwl(scpi_t * context) {

    SCPI_ResultMnemonic(context, "20M");

    return SCPI_RES_OK;
}

static scpi_result_t query_inv(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
    
}


static scpi_result_t query_fft_split(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
    
}

static scpi_result_t query_fft_unit(scpi_t * context) {

    SCPI_ResultMnemonic(context, "DB");

    return SCPI_RES_OK;
    
}


static scpi_result_t query_fft_source(scpi_t * context) {

    SCPI_ResultMnemonic(context, "CHAN1");

    return SCPI_RES_OK;
    
}





static scpi_result_t query_math_disp(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
    
}




static scpi_result_t query_unit(scpi_t * context) {

    SCPI_ResultMnemonic(context, "VOLTAGE");

    return SCPI_RES_OK;
}


static scpi_result_t query_vern(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
}

static scpi_result_t query_timebase_delay(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
}


static scpi_result_t query_timebase_offset(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0.0");

    return SCPI_RES_OK;
}

static scpi_result_t time_mode(scpi_t * context) {

    SCPI_ResultMnemonic(context, "XY");

    return SCPI_RES_OK;
}



static scpi_result_t query_timebase_scale(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0.0");

    return SCPI_RES_OK;
}




static scpi_result_t query_wav_yor(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
}

static scpi_result_t query_wav_xor(scpi_t * context) {

    SCPI_ResultMnemonic(context, "0");

    return SCPI_RES_OK;
}




/*
**
 * *ESE
 * @param context
 * @return 
 *
scpi_result_t SCPI_CoreEse(scpi_t * context) {
    int32_t new_ESE;
    if (SCPI_ParamInt32(context, &new_ESE, TRUE)) {
        SCPI_RegSet(context, SCPI_REG_ESE, (scpi_reg_val_t) new_ESE);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}
*/


void set_mem_depth(int depth);


static scpi_result_t set_acq_mem(scpi_t * context) {


    if (SCPI_ParamInt32(context, &memLen, TRUE)) {
        printf("NEW mem depth %d\r\n",memLen);
        set_mem_depth(memLen);
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}



static scpi_result_t query_acq_type(scpi_t * context) {

     SCPI_ResultMnemonic(context, "NORM");

    return SCPI_RES_OK;

}

// Arearageing
static scpi_result_t query_acq_averages(scpi_t * context) {

     SCPI_ResultMnemonic(context, "2");

    return SCPI_RES_OK;

}

static scpi_result_t autoscale(scpi_t * context) {


    printf("AUTOSCALE\n");

    #ifdef START_SAMPLE_TASK
    //start_sampling();
    #endif

    return SCPI_RES_OK;

}







static scpi_result_t query_acq_mem(scpi_t * context) {

    //SCPI_ResultMnemonic(context, "1400");

    SCPI_ResultInt32(context, memLen);


    return SCPI_RES_OK;

}


//const char *list[]={"BYTE"};

static scpi_result_t set_wav_form(scpi_t * context) {
    char *result;
    size_t len;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}

// Trigger mode is set to single
static scpi_result_t set_single_acq(scpi_t * context) {

times_called=0;

printf("SINGLE\n");
#ifdef START_SAMPLE_TASK
start_sampling(true);
#endif

   return SCPI_RES_OK;

}

// NORM, MAXimum,RAW (No  BYTE or WORD)??
static scpi_result_t set_wav_mode(scpi_t * context) {
    char *result;
    size_t len;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}


static scpi_result_t wav_end(scpi_t * context) {
    //char *result;
    //size_t len;
    //if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
    //    return SCPI_RES_OK;
    //}

    return SCPI_RES_OK;
}



extern void setTimescale(float scale);

// Time scale
static scpi_result_t query_samplerate(scpi_t * context) {
    SCPI_ResultMnemonic(context, "140000");

    return SCPI_RES_OK;

}

// Time scale
static scpi_result_t query_acq_mode(scpi_t * context) {
    SCPI_ResultMnemonic(context, "AUTO");

    return SCPI_RES_OK;

}





// Time scale
static scpi_result_t set_time_scale(scpi_t * context) {
    
    if (SCPI_ParamFloat(context, &time_scale_value,TRUE)) {

        printf("Time scale %.6f", time_scale_value);
        setTimescale(time_scale_value);

        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}
/*
WAVeform:DATA?Command Format::WAVeform:DATA?
[  <source>]
Function:The command reads waveform data from the specified source. 
<source> may be:  CHANnel1, CHANnel2,  CHANnel3,  CHANnel4 or   MATH. 
Returned Format:     The query returns a certain amount of waveform data that specifed by :WAVeform:POINts.


 NOTE:The command returns the data on the screen when the waveforms are played back. At 
RIGOLCommand SystemsProgramming  Guide  for  DS1000B  Series 2-70this moment, only NORMal and MAXimum mode are available and the system is inSTOP state

this moment, only NORMal and MAXimum mode are available and the system is inSTOP state.
600 points are returned in common operation（+, －, ×）while 500 points are returned in FFT operation in all modes (NORMal, RAW, MAXimum). 
The waveform data read in NORMal mode are fixed to be 600 points in STOP state.
 In the condition when increasing the time base in STOP state to make all the waveforms display on the screen, 
 you may find that some invalid data may be contained in data returned. 
 So, you are recommended to read the data in RAW mode while in STOPstate.
 Example::WAV:DATA?CHAN1
 Read the data from CH1. 
 
 
 :WAVeform:POINtsCommand Format::WAVeform:POINts <points> 
 :WAVeform:POINts?Function:
 This command sets the number of waveform points that are required to be returned. 
 The default is 0. <points> has different value ranges in different modes.
 NORMal: 0  ~600
 RAW: 0~8192 or 0~16384 (in half channel state) 
 NOTE: Half channel indicates selecting either one of the channels from CH1 and CH2, 
 orfrom CH3 and CH4.Returned Format: The query returns an integer, for example: 10. 
 
 NOTE:If  you set the number of waveform points to be 0, the query will return the maximum points in current mode (NORMal: return 600 points, RAW: return current memory depth); 
 In MATH operation, 600 points are returned no matter what mode it is;In FFT, the maximum points will always be 500.
Command Systems                                                                                                                      RIGOL Programming Guide for DS1000B Series2-71
Example::WAV:POIN 20        
Set the waveform points as 20.:WAV:POIN?        
Return 20.For details about storage format of waveform points,

*/


// https://www.batronix.com/files/Rigol/Oszilloskope/_DS&MSO2000A/MSO2000A_DS2000A_ProgrammingGuide_EN.pdf
// Page 2-364
/*
Note: When LA is set as the channel source of waveform data reading, the query always returns waveform data in
 WORD format. The statuses of one group of digital signals are represented by two bytes. Of which, for the first byte, 
its highest bit to the lowest bit respectively corresponds to the 
 state of the digital channel from D7 to D0; 
 for the second byte, its highest bit to the lowest bit respectively corresponds to the state 
 of the digital channel from D15 to D8
*/

// :WAV:SOUR CHAN1
// :WAV:SOUR LA
static scpi_result_t set_wav_source(scpi_t * context) {
    char *result;
    size_t len;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        if (strncmp(result,"LA",2)==0) {
            readDigital=true;
        } else {
            readDigital=false;
        }

        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}


static scpi_result_t wav_read_reset(scpi_t * context) {

    wav_read_position=0;
    return SCPI_RES_OK;
}

static scpi_result_t wav_read_begin(scpi_t * context) {

    wav_read_position=0;
    return SCPI_RES_OK;
}

static scpi_result_t query_trig_edge_level(scpi_t * context) {
    SCPI_ResultMnemonic(context, "0.0");

    return SCPI_RES_OK;
}

static scpi_result_t query_trig_holdoff(scpi_t * context) {
    SCPI_ResultMnemonic(context, "0.0001");

    return SCPI_RES_OK;
}



static scpi_result_t set_trig_edge_level(scpi_t * context) {
   char *result;
    size_t len;
    printf("TRIG ");
 
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}

 

static scpi_result_t set_trig_edge_source(scpi_t * context) {
   char *result;
    size_t len;
    printf("TRIG ");
 
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        if (strncmp(result,"D0",2)==0) 
        {
            printf("D0\n");
            trig_pin=0;
        } else {
            trig_pin=atoi(&result[1]);
            printf("other %d\n",trig_pin);
        }

        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}


static scpi_result_t set_wav_points(scpi_t * context) {
    char *result;
    size_t len;
    int32_t num_points=0;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        num_points=atoi(result);

        //SCPI_ResultInt32(context, num_points);

        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}



// ON or OFF
static scpi_result_t chan_la_disp0(scpi_t * context) {
    char *result;
    size_t len;
    if (SCPI_ParamCharacters(context, &result, &len,TRUE)) {
        return SCPI_RES_OK;
    }

    return SCPI_RES_OK;
}


const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = SCPI_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = My_CoreTstQ,},
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    {.pattern = ":DISP:GRID?", .callback = query_disp_grid,},
    {.pattern = ":DISP:GRAD:TIME?", .callback = query_disp_grad_time,},

    {.pattern = ":DISP:TYPE?", .callback = query_disp_type,},


    { .pattern = "CHAN1:DISP?", .callback = chan_disp_on,},
    { .pattern = "CHAN2:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN3:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN4:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN5:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN6:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN7:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN8:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN9:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN10:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN11:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN12:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN13:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN14:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN15:DISP?", .callback = chan_disp_off,},
    { .pattern = "CHAN16:DISP?", .callback = chan_disp_off,},

    { .pattern = "LA:DISP?", .callback = chan_disp_on,},
    { .pattern = "DIG0:TURN?", .callback = chan_disp_on,},
    { .pattern = "DIG1:TURN?", .callback = chan_disp_on,},
    { .pattern = "DIG2:TURN?", .callback = chan_disp_on,},
    { .pattern = "DIG3:TURN?", .callback = chan_disp_on,},
    { .pattern = "DIG4:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG5:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG6:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG7:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG8:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG9:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG10:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG11:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG12:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG13:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG14:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG15:TURN?", .callback = chan_disp_off,},
    { .pattern = "DIG16:TURN?", .callback = chan_disp_off,},

    { .pattern = ":TIM:SCAL?", .callback = time_scale,},
    { .pattern = ":CHAN1:PROB?", .callback = chan1_probe,},
    { .pattern = ":CHAN2:PROB?", .callback = chan1_probe,},
    { .pattern = ":CHAN1:SCAL", .callback = set_chan1_scal,},
    { .pattern = ":CHAN2:SCAL", .callback = set_chan2_scal,},


    { .pattern = ":CHAN1:SCAL?", .callback = chan1_scal,},
    { .pattern = ":CHAN2:SCAL?", .callback = chan2_scal,},

    { .pattern = "CHAN1:OFFS?", .callback = chan1_offset,},
    { .pattern = "CHAN2:OFFS?", .callback = chan1_offset,},

    { .pattern = "CHAN1:COUP?", .callback = chan1_coup,},
    { .pattern = "CHAN2:COUP?", .callback = chan1_coup,},

    { .pattern = "TRIG:EDG:LEV?", .callback = query_trig_edge_level,},
    { .pattern = "TRIG:EDG:LEV", .callback = set_trig_edge_level,},

    { .pattern = ":TRIG:HOLD?", .callback = query_trig_holdoff,},



    { .pattern = "TRIG:EDGE:SOUR?", .callback = trig_edge_source,},
    { .pattern = "TIM:OFFS?", .callback = tim_offset,},
    { .pattern = "TRIG:EDGE:SLOP?", .callback = trig_edge_slope,},
    { .pattern = "TRIG:EDGE:LEV?", .callback = trig_edge_level,},

    { .pattern = ":LA:STAT?", .callback = la_stat_query,},

    { .pattern = ":TIM:SCAL", .callback = set_time_scale,},

    { .pattern = ":ACQ:SRAT?", .callback = query_samplerate,},
    { .pattern = ":ACQ:MODE?", .callback = query_acq_mode,},


    { .pattern = ":LA:DIG0:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG1:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG2:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG3:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG4:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG5:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG6:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG7:DISP?", .callback = chan_disp_on,},
    { .pattern = ":LA:DIG8:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG9:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG10:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG11:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG12:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG13:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG14:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG15:DISP?", .callback = chan_disp_off,},
    { .pattern = ":LA:DIG16:DISP?", .callback = chan_disp_off,},


   { .pattern = ":LA:DIG0:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG1:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG2:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG3:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG4:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG5:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG7:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG8:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG9:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG10:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG11:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG12:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG13:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG14:DISP", .callback = chan_la_disp0,},
   { .pattern = ":LA:DIG15:DISP", .callback = chan_la_disp0,},
    
    { .pattern = "WAV:YREF?", .callback = wav_yref,},
    { .pattern = "WAV:STAT?", .callback = wav_stat,},
    { .pattern = "WAV:FORM", .callback = set_wav_form,},
    { .pattern = ":WAV:MODE", .callback = set_wav_mode,},
    { .pattern = ":WAV:END", .callback = wav_end,},


    { .pattern = ":MATH:FFT:SPL?", .callback = query_fft_split,},
    { .pattern = ":MATH:FFT:UNIT?", .callback = query_fft_unit,},
    { .pattern = ":MATH:FFT:SOUR?", .callback = query_fft_source,},


    { .pattern = ":MATH:DISP?", .callback = query_math_disp,},




    { .pattern = ":SING", .callback = set_single_acq,},

    { .pattern = "TRIG:EDGE:SOUR", .callback = set_trig_edge_source,},
    // NORM
    { .pattern = "WAV:SOUR", .callback = set_wav_source,},
    { .pattern = ":WAV:RES", .callback = wav_read_reset,},
    { .pattern = ":WAV:BEG", .callback = wav_read_begin,},

    // 
    { .pattern = "WAV:POIN", .callback = set_wav_points,},

    { .pattern = ":WAV:YINC?", .callback = query_wav_yinc,},
    { .pattern = ":WAV:YOR?", .callback = query_wav_yor,},
    { .pattern = ":WAV:XOR?", .callback = query_wav_xor,},


    { .pattern = ":CHAN1:BWL?", .callback = query_bwl,},
    { .pattern = ":CHAN2:BWL?", .callback = query_bwl,},
    

    { .pattern = ":CHAN1:INV?", .callback = query_inv,},
    { .pattern = ":CHAN2:INV?", .callback = query_inv,},

    { .pattern = ":CHAN1:UNIT?", .callback = query_unit,},
    { .pattern = ":CHAN2:UNIT?", .callback = query_unit,},

    { .pattern = ":CHAN1:VERN?", .callback = query_vern,},
    { .pattern = ":CHAN2:VERN?", .callback = query_vern,},
    

    { .pattern = ":TIM:DEL:ENAB?", .callback = query_timebase_delay,},

    { .pattern = ":TIM:DEL:OFFS?", .callback = query_timebase_offset,},
    { .pattern = ":TIM:DEL:SCAL?", .callback = time_scale,},
    { .pattern = ":TIM:MODE?", .callback = time_mode,},


    { .pattern = "RUN", .callback = run_to_the_hills,},
    { .pattern = "STOP", .callback = stop_acquisition,},
    { .pattern = "TRIG:STAT?", .callback = trig_status,},

    { .pattern = "TRIG:COUP?", .callback = query_trig_coup,},

    { .pattern = "TRIG:SWE?", .callback = query_trig_sweep,},
    { .pattern = "TRIG:SWE", .callback =  trig_swe,},

    { .pattern = "TRIG:MODE?", .callback = query_trig_mode,},
    { .pattern = "TRIG:SLOP?", .callback = query_trig_slope,},

    { .pattern = "TRIG:EDG:SLOP?", .callback = query_trig_edg_slope,},
    { .pattern = "TRIG:EDG:SLOP", .callback = set_trig_edg_slope,},


    { .pattern = "TRIG:EDG:SOUR?", .callback = query_trig_edg_source,},




    { .pattern = "TRIG:EDG:SOUR", .callback = trig_edg_source,},
    

//sr: [00:53.341407] scpi_tcp: Successfully sent SCPI command: ':RUN'.
// Operation comlete...?
//sr: [00:53.341493] scpi_tcp: Successfully sent SCPI command: '*OPC?'.
//sr: [00:53.384473] scpi: Got response: '1', length 1.
//sr: [00:53.384551] std: rigol-ds: Starting acquisition.
//sr: [00:53.384580] std: rigol-ds: Sending SR_DF_HEADER packet.
//sr: [00:53.384602] session: bus: Received SR_DF_HEADER packet.
//sr: [00:53.384666] hwdriver: sr_config_get(): key 30000 (samplerate) sdi 0x55e5a66914b0 cg NULL -> uint64 73
//sr: [00:53.384694] rigol-ds: Starting data capture for frameset 1 of 1
//sr: [00:53.384711] session: bus: Received SR_DF_FRAME_BEGIN packet.
//sr: [00:53.435286] scpi_tcp: Successfully sent SCPI command: ':TRIG:STAT?'.

    {.pattern = "WAV:DATA?", .callback = wav_data,},  // CHAN1


// Not used!
    { .pattern = "D0:DISP?", .callback = chan_disp_on,},
    { .pattern = "D1:DISP?", .callback = chan_disp_on,},
    { .pattern = "D2:DISP?", .callback = chan_disp_on,},
    { .pattern = "D3:DISP?", .callback = chan_disp_off,},
    { .pattern = "D4:DISP?", .callback = chan_disp_off,},
    { .pattern = "D5:DISP?", .callback = chan_disp_off,},
    { .pattern = "D6:DISP?", .callback = chan_disp_off,},
    { .pattern = "D7:DISP?", .callback = chan_disp_off,},
    { .pattern = "D8:DISP?", .callback = chan_disp_off,},
    { .pattern = "D9:DISP?", .callback = chan_disp_off,},
    { .pattern = "D10:DISP?", .callback = chan_disp_off,},
    { .pattern = "D11:DISP?", .callback = chan_disp_off,},
    { .pattern = "D12:DISP?", .callback = chan_disp_off,},
    { .pattern = "D13:DISP?", .callback = chan_disp_off,},
    { .pattern = "D14:DISP?", .callback = chan_disp_off,},
    { .pattern = "D15:DISP?", .callback = chan_disp_off,},
    //ACQ:MDEP?          //asks scope for mem depth or number of points. 
    // https://rigol.desk.com/customer/en/portal/articles/2726897-deep-memory-binary-data-collection-and-conversion
    // https://www.codeproject.com/Articles/869421/Interfacing-Rigol-Oscilloscopes-with-C
    { .pattern = "ACQ:MDEP", .callback = set_acq_mem,},
    { .pattern = "ACQ:MDEP?", .callback = query_acq_mem,},
    { .pattern = ":ACQuire:TYPE?", .callback = query_acq_type,},

    { .pattern = ":ACQuire:AVERages?", .callback = query_acq_averages,},




    {.pattern = "AUT", .callback = autoscale,},


    
    
    /* {.pattern = "STATus:OPERation?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:EVENt?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:CONDition?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle?", .callback = scpi_stub_callback,}, */

    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},

    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    /* DMM */
    {.pattern = "MEASure:VOLTage:DC?", .callback = DMM_MeasureVoltageDcQ,},
    {.pattern = "CONFigure:VOLTage:DC", .callback = DMM_ConfigureVoltageDc,},
    {.pattern = "MEASure:VOLTage:DC:RATio?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:VOLTage:AC?", .callback = DMM_MeasureVoltageAcQ,},
    {.pattern = "MEASure:CURRent:DC?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:CURRent:AC?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:RESistance?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:FRESistance?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:FREQuency?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:PERiod?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:COUNt:SOURce?", .callback = measure_count_source,},

    {.pattern = "SYSTem:COMMunication:TCPIP:CONTROL?", .callback = SCPI_SystemCommTcpipControlQ,},

    {.pattern = "TEST:BOOL", .callback = TEST_Bool,},
    {.pattern = "TEST:CHOice?", .callback = TEST_ChoiceQ,},
    {.pattern = "TEST#:NUMbers#", .callback = TEST_Numbers,},
    {.pattern = "TEST:TEXT", .callback = TEST_Text,},
    {.pattern = "TEST:ARBitrary?", .callback = TEST_ArbQ,},
    {.pattern = "TEST:CHANnellist", .callback = TEST_Chanlst,},

 
    


   

    SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;
