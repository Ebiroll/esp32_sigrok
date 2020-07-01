#ifndef __SCPI_DEF_H_
#define __SCPI_DEF_H_

#include "scpi/scpi.h"

#define SCPI_INPUT_BUFFER_LENGTH 256
#define SCPI_ERROR_QUEUE_SIZE 17

#define SCPI_IDN1 "RIGOL TECHNOLOGIES"
//#define   SCPI_IDN1 "RIGOL TECHNOLOGIES,DS1102E,DS1EB104702974,00.02.05.01.00"
//# firmware < 0.2.4, using raw data format.
//#define SCPI_IDN2 "VS5022D"     
//#define SCPI_IDN3 "VS50234567"
//#define SCPI_IDN4 "00.02.03.03.00"
// 00.02.04
//#define SCPI_IDN1 "Rigol Technologies,MSO2302A,DS1EXXXXXXXXXX,00.02.05.02.00"
#define SCPI_IDN2 "MSO2302A"     
#define SCPI_IDN3 "DS1EXXXXXXXXXX"
#define SCPI_IDN4 "00.02.05.02.00"


extern const scpi_command_t scpi_commands[];
extern scpi_interface_t scpi_interface;
extern char scpi_input_buffer[];
extern scpi_error_t scpi_error_queue_data[];
extern scpi_t scpi_context;

size_t SCPI_Write(scpi_t * context, const char * data, size_t len);
int SCPI_Error(scpi_t * context, int_fast16_t err);
scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
scpi_result_t SCPI_Reset(scpi_t * context);
scpi_result_t SCPI_Flush(scpi_t * context);


scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context);

#endif /* __SCPI_DEF_H_ */

