#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__

#define DEBUG_EN

#define USE_BOSCH_SENSOR_API


#define USE_BMI270

#if defined(USE_BMI270)
//#define ACC_SELFTEST
//#define GYRO_SELFTEST
//#define ACC_GYRO_SELFTEST
//#define CRT
//#define ACC_ONLY
//#define GYRO_ONLY
#define ACC_GYRO
	#if defined(ACC_ONLY) || defined(ACC_GYRO)
	#define STEP_COUNTER
	//#define WRIST_WEAR_WAKE_UP
	#endif
/*Only support to select one*/
#define FIFO_POLL
//#define FIFO_WM_INT
#define SUPPORT_LOWPOWER
#endif


#endif
