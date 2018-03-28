#include <stdbool.h>
#include <rtthread.h>
#include <stm32f4xx.h>
#include "drv_led.h"


/*led 驱动代码*/

/*LED 极性*/
#define LED_POL_POS 0
#define LED_POL_NEG 1

static bool isInit = false;

typedef struct 
{
	GPIO_TypeDef* GPIOx;
	uint16_t pin;
	int polarity;
}led_t;

static led_t leds[LED_NUM] =
{
	[LED_BLUE_L]	= {GPIOB, GPIO_PIN_12, LED_POL_POS},
	[LED_GREEN_L]	= {GPIOA, GPIO_PIN_6,  LED_POL_NEG},
	[LED_RED_L] 	= {GPIOA, GPIO_PIN_7,  LED_POL_NEG},
	[LED_GREEN_R]	= {GPIOC, GPIO_PIN_13, LED_POL_NEG},
	[LED_RED_R] 	= {GPIOC, GPIO_PIN_14, LED_POL_NEG},
};

/* LED初始化 */
int ledInit(void)
{
	if(isInit)	return 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;

	/*使能led时钟*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/*LED_GREEN_L PA6	LED_RED_L PA7*/
	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
//    GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	/*LED_BLUE_L PB12*/
	GPIO_InitStructure.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*LED_GREEN_R PC13	LED_RED_R PC14*/
	GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	ledClearAll();
	
	isInit = true;
	return 0;
	
}
INIT_BOARD_EXPORT(ledInit);

/* LED测试 */
bool ledTest(void)
{
	ledSet(LED_GREEN_L, 1);
	ledSet(LED_GREEN_R, 1);
	ledSet(LED_RED_L, 0);
	ledSet(LED_RED_R, 0);
	rt_thread_delay(250);
	//delay_xms(250);
	
	ledSet(LED_GREEN_L, 0);
	ledSet(LED_GREEN_R, 0);
	ledSet(LED_RED_L, 1);
	ledSet(LED_RED_R, 1);

	//delay_xms(250);
	rt_thread_delay(250);

	ledClearAll();
	ledSet(LED_BLUE_L, 1);

	return isInit;
}

/*关闭所有LED*/
void ledClearAll(void)
{	
	for(uint8_t i = 0; i < LED_NUM; i++)
	{
		ledSet((led_e)i, 0);
	}
}

/*打开所有LED*/
void ledSetAll(void)
{
	for(uint8_t i = 0; i < LED_NUM; i++)
	{
		ledSet((led_e)i, 1);
	}
}
/*LED闪烁1次*/
void ledFlashOne(led_e led, uint32_t onTime, uint32_t offTime)
{
	ledSet(led, 1);
	rt_thread_delay(onTime);
	//delay_xms(onTime);
	ledSet(led, 0);
	rt_thread_delay(offTime);
	//delay_xms(offTime);
}

/* 设置某个LED的状态 */
void ledSet(led_e led, bool value)
{
	if (led>LED_NUM)
		return;
	
	if (leds[led].polarity == LED_POL_NEG)
		value = !value;

	HAL_GPIO_WritePin(leds[led].GPIOx, leds[led].pin, value);
//	if (leds[led].polarity == LED_POL_NEG)
//		value = !value;

//	if(value)
//		GPIO_SetBits(leds[led].port, leds[led].pin);
//	else
//		GPIO_ResetBits(leds[led].port, leds[led].pin); 
}


