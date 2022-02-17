#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32l4xx_hal.h"
#include "common_porting.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;


uint8_t GTXBuffer[512], GRXBuffer[2048];

volatile uint8_t int1_flag = 0;
volatile uint8_t int2_flag = 0;



#if defined(FIFO_WM_INT)
extern volatile uint16_t fifo_read_ready;
extern volatile uint16_t bma425_fifo_ready;
extern volatile uint16_t bma400fifo_ready;
extern volatile uint16_t bmi160_fifo_ready;
#endif


void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}

void UART_Printf(uint8_t* buff, uint16_t size)
{
    //HAL_UART_Transmit_DMA(&huart2, buff, size);
    HAL_UART_Transmit(&UART_HANDLE, buff, size, BUS_TIMEOUT);
}

char chBuffer[512];
void PDEBUG(char *format, ...)
{
#if defined(DEBUG_EN)
    va_list ap;
    //char timestamp[16];
    va_start(ap, format);
    vsnprintf(chBuffer, sizeof(chBuffer), format, ap);
    //sprintf(timestamp, "[%d]", xTaskGetTickCount()); //xTaskGetTickCountFromISR()
    //Printf((uint8_t *)timestamp, strlen(timestamp));
    UART_Printf((uint8_t *)chBuffer,strlen(chBuffer));
    va_end(ap);
#endif
}

#if defined(USE_BOSCH_SENSOR_API)
int8_t SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	// send register address
	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &reg_addr, 1, BUS_TIMEOUT);
	HAL_I2C_Master_Receive(&I2C_HANDLE, DevAddress, reg_data, len, BUS_TIMEOUT);
	return 0;
}

int8_t SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	GTXBuffer[0] = reg_addr;
	memcpy(&GTXBuffer[1], reg_data, len);

	// send register address
	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, GTXBuffer, len+1, BUS_TIMEOUT);
	return 0;
}


#endif


