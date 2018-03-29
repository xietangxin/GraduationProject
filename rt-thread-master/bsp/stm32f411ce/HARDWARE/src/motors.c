#include <rtthread.h>


#include "motors.h"
#include "pm.h"
#include "stm32f4xx.h"



/* �����������	*/



static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };


static uint16_t ratioToCCRx(uint16_t val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void motorsInit(void)	/*�����ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_HandleTypeDef TIM2_Handler;
	TIM_HandleTypeDef TIM4_Handler;
	TIM_OC_InitTypeDef TIM_OCHandler;
	
	//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��PORTA PORTBʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4,ENABLE);  	//TIM2��TIM4ʱ��ʹ��    
	
//	TIM_DeInit(TIM4);	//���³�ʼ��TIM4ΪĬ��״̬
//	TIM_DeInit(TIM2);	//���³�ʼ��TIM2ΪĬ��״̬
	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 	//PB7 ����ΪTIM4 CH2	MOTOR1
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 	//PB6 ����ΪTIM4 CH1	MOTOR2
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2); 	//PB10����ΪTIM2 CH3	MOTOR3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM2); 	//PA5 ����ΪTIM2 CH1	MOTOR4
	
	
	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	
	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10;	//PB6 7 10
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//���ù���
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//���츴�����
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//����
//	GPIO_Init(GPIOB,&GPIO_InitStructure);              				//��ʼ��PB6 7 10
	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;							//PA5
//	GPIO_Init(GPIOA,&GPIO_InitStructure);              				//��ʼ��PA5		
	
	TIM2_Handler.Instance = TIM2;
	TIM2_Handler.Init.Prescaler = MOTORS_PWM_PRESCALE;
	TIM2_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM2_Handler.Init.Period = MOTORS_PWM_PERIOD;
	TIM2_Handler.Init.ClockDivision = 0;
	HAL_TIM_PWM_Init(&TIM2_Handler);
	
	TIM4_Handler = TIM2_Handler;
	TIM4_Handler.Instance = TIM4;
	HAL_TIM_PWM_Init(&TIM4_Handler);
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ	
//	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//ʱ�ӷ�Ƶ
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//�ظ���������
//	
//	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//��ʼ��TIM4
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
	TIM_OCHandler.OCMode = TIM_OCMODE_PWM1;
	TIM_OCHandler.Pulse = 0;
	TIM_OCHandler.OCPolarity = TIM_OCPOLARITY_LOW;
	HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &TIM_OCHandler, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &TIM_OCHandler, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM_OCHandler, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM_OCHandler, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_2);
		

	
//	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
//	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
//	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
//	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�ߵ�ƽ��Ч
//	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ	
//	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
//	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
//	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�
//	
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
//	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
//	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
// 
//	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPEʹ�� 
//	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPEʹ�� 
	
//	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
//	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	

	isInit = true;
}

/*�������*/
bool motorsTest(void)
{
	int i;

	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{		
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		
		rt_thread_delay(rt_tick_from_millisecond(MOTORS_TEST_ON_TIME_MS));
		//delay_xms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
		
		rt_thread_delay(rt_tick_from_millisecond(MOTORS_TEST_ON_TIME_MS));
		//delay_xms(MOTORS_TEST_DELAY_TIME_MS);
	}

	return isInit;
}

/*���õ��PWMռ�ձ�*/
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
	if (isInit) 
	{
		uint16_t ratio=ithrust;

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
			case 0:		/*MOTOR_M1*/
				TIM4->CCR2 = ratioToCCRx(ratio);
				//TIM_SetCompare2(TIM4,ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				TIM4->CCR1 = ratioToCCRx(ratio);
				//TIM_SetCompare1(TIM4,ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				TIM2->CCR3 = ratioToCCRx(ratio);
				//TIM_SetCompare3(TIM2,ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM2->CCR1 = ratioToCCRx(ratio);
				//TIM_SetCompare1(TIM2,ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}

