#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32l4xx_hal.h"
#include "common_porting.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;



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



#endif


