#include "motors.h"
#include "pm.h"
#include "stm32f4xx.h"

#include <rtthread.h>

/* 电机驱动代码	*/

static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };


static uint16_t ratioToCCRx(uint16_t val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

/* 电机初始化 */
void motorsInit(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_HandleTypeDef TIM2_Handler; 	
	TIM_HandleTypeDef TIM4_Handler;
	TIM_OC_InitTypeDef TIM_OCHandler;
		
	/* 初始化时钟 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
		
	/* PB7 PB6 复用为TIM4 CH2 CH1 */
	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* PB10 复用为 TIM2 CH3 */
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* PA5 复用为TIM2 CH1 */
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 初始化TIM设置 回调函数为HAL_TIM_PWM_MspInit() */
	TIM2_Handler.Instance = TIM2;
	TIM2_Handler.Init.Prescaler = MOTORS_PWM_PRESCALE;
	TIM2_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM2_Handler.Init.Period = MOTORS_PWM_PERIOD;
	TIM2_Handler.Init.ClockDivision = 0;
	HAL_TIM_PWM_Init(&TIM2_Handler);
	
	TIM4_Handler = TIM2_Handler;
	TIM4_Handler.Instance = TIM4;
	HAL_TIM_PWM_Init(&TIM4_Handler);
	
	/* 设置TIM的PWM模式 */
	TIM_OCHandler.OCMode = TIM_OCMODE_PWM1;
	TIM_OCHandler.Pulse = 0;
	TIM_OCHandler.OCPolarity = TIM_OCPOLARITY_LOW;
	HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &TIM_OCHandler, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &TIM_OCHandler, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM_OCHandler, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM_OCHandler, TIM_CHANNEL_2);
	
	/* 使能TIM */
	HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_2);
		
	isInit = true;
}

/*电机测试*/
bool motorsTest(void)
{
	int i;

	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{		
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		
		rt_thread_delay(rt_tick_from_millisecond(MOTORS_TEST_ON_TIME_MS));
		motorsSetRatio(MOTORS[i], 0);
		
		rt_thread_delay(rt_tick_from_millisecond(MOTORS_TEST_ON_TIME_MS));
	}

	return isInit;
}

/*设置电机PWM占空比*/
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
	if (isInit) 
	{
		uint16_t ratio=ithrust;

	/* 电池油门补偿 */
	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		float thrust = ((float)ithrust / 65536.0f) * 60;
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
		float supply_voltage = pmGetBatteryVoltage();
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		motor_ratios[id] = ratio;		
	#endif
		
		switch(id)
		{
			case 0:		
				TIM4->CCR2 = ratioToCCRx(ratio);
				break;
			case 1:		
				TIM4->CCR1 = ratioToCCRx(ratio);
				break;
			case 2:		
				TIM2->CCR3 = ratioToCCRx(ratio);
				break;
			case 3:			
				TIM2->CCR1 = ratioToCCRx(ratio);
				break;
			default: break;
		}	
	}
}

