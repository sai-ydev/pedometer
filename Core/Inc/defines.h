/*
 * defines.h
 *
 *  Created on: Feb 16, 2022
 *      Author: e8pk22
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

/**
 * @defgroup TM_DELAY_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Custom timer structure
 */
typedef struct _TM_DELAY_Timer_t {
	uint32_t ARR;                                        /*!< Auto reload value */
	uint32_t AR;                                         /*!< Set to 1 if timer should be auto reloaded when it reaches zero */
	uint32_t CNT;                                        /*!< Counter value, counter counts down */
	uint8_t CTRL;                                        /*!< Set to 1 when timer is enabled */
	void (*Callback)(struct _TM_DELAY_Timer_t*, void *); /*!< Callback which will be called when timer reaches zero */
	void* UserParameters;                                /*!< Pointer to user parameters used for callback function */
} TM_DELAY_Timer_t;

/**
 * @}
 */

/**
 * @defgroup TM_DELAY_Variables
 * @brief    Library variables
 * @{
 */
extern __IO uint32_t TM_Time2;
extern __IO uint32_t TM_Time;
/**
 * @}
 */

/**
 * @defgroup TM_DELAY_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes delay functions
 * @param  None
 * @retval DWT counter start status
 *           - 0: DWT counter did not start, delay for microseconds won't work
 *           - > 0: DWT counter works OK, delay is ready to use
 */
uint32_t TM_DELAY_Init(void);

/**
 * @brief  Delays for amount of micro seconds
 * @param  micros: Number of microseconds for delay
 * @retval None
 */
__STATIC_INLINE void Delay(__IO uint32_t micros) {
#if !defined(STM32F0xx)
	uint32_t start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	micros *= (HAL_RCC_GetHCLKFreq() / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - start) < micros);
#else
	/* Go to clock cycles */
	micros *= (SystemCoreClock / 1000000) / 5;

	/* Wait till done */
	while (micros--);
#endif
}

/**
 * @brief  Delays for amount of milli seconds
 * @param  millis: Number of milliseconds for delay
 * @retval None
 */
__STATIC_INLINE void Delayms(uint32_t millis) {
	/* Use HAL library for delay ms purpose */
	HAL_Delay(millis);
}

/**
 * @brief  Gets the TM_Time variable value
 * @param  None
 * @retval Current time in milliseconds
 */
#define TM_DELAY_Time()					(TM_Time)

/**
 * @brief  Sets value for TM_Time variable
 * @param  time: Time in milliseconds
 * @retval None
 */
#define TM_DELAY_SetTime(time)			(TM_Time = (time))

/**
 * @brief  Gets the TM_Time2 variable value
 * @param  None
 * @retval Current time in milliseconds
 * @note   This is not meant for public use
 */
#define TM_DELAY_Time2()				(TM_Time2)

/**
 * @brief  Sets value for TM_Time variable
 * @param  time: Time in milliseconds
 * @retval None
 * @note   This is not meant for public use
 */
#define TM_DELAY_SetTime2(time)			(TM_Time2 = (time))

/**
 * @defgroup TM_DELAY_Timer_Functions
 * @brief    Software timer functions
 * @{
 */

/**
 * @brief  Creates a new custom timer which has 1ms resolution
 * @note   It uses @ref malloc for memory allocation for timer structure
 * @param  ReloadValue: Number of milliseconds when timer reaches zero and callback function is called
 * @param  AutoReloadCmd: If set to 1, timer will start again when it reaches zero and callback is called
 * @param  StartTimer: If set to 1, timer will start immediately
 * @param  *TM_DELAY_CustomTimerCallback: Pointer to callback function which will be called when timer reaches zero
 * @param  *UserParameters: Pointer to void pointer to user parameters used as first parameter in callback function
 * @retval Pointer to allocated timer structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerCreate(uint32_t ReloadValue, uint8_t AutoReloadCmd, uint8_t StartTimer, void (*TM_DELAY_CustomTimerCallback)(struct _TM_DELAY_Timer_t*, void *), void* UserParameters);

/**
 * @brief  Deletes already allocated timer
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @retval None
 */
void TM_DELAY_TimerDelete(TM_DELAY_Timer_t* Timer);

/**
 * @brief  Stops custom timer from counting
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @retval Pointer to @ref TM_DELAY_Timer_t structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerStop(TM_DELAY_Timer_t* Timer);

/**
 * @brief  Starts custom timer counting
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @retval Pointer to @ref TM_DELAY_Timer_t structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerStart(TM_DELAY_Timer_t* Timer);

/**
 * @brief  Resets custom timer counter value
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @retval Pointer to @ref TM_DELAY_Timer_t structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerReset(TM_DELAY_Timer_t* Timer);

/**
 * @brief  Sets auto reload feature for timer
 * @note   Auto reload features is used for timer which starts again when zero is reached if auto reload active
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @param  AutoReload: Set to 1 if you want to enable AutoReload or 0 to disable
 * @retval Pointer to @ref TM_DELAY_Timer_t structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerAutoReloadCommand(TM_DELAY_Timer_t* Timer, uint8_t AutoReloadCmd);

/**
 * @brief  Sets auto reload value for timer
 * @param  *Timer: Pointer to @ref TM_DELAY_Timer_t structure
 * @param  AutoReloadValue: Value for timer to be set when zero is reached and callback is called
 * @note   AutoReload feature must be enabled for timer in order to get this to work properly
 * @retval Pointer to @ref TM_DELAY_Timer_t structure
 */
TM_DELAY_Timer_t* TM_DELAY_TimerAutoReloadValue(TM_DELAY_Timer_t* Timer, uint32_t AutoReloadValue);

/**
 * @}
 */

/**
 * @brief  User function, called each 1ms when interrupt from timer happen
 * @note   Here user should put things which has to be called periodically
 * @param  None
 * @retval None
 * @note   With __weak parameter to prevent link errors if not defined by user
 */
void TM_DELAY_1msHandler(void);

/**
 * @}
 */

#endif /* INC_DEFINES_H_ */
