/*
* File      : drv_usart.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2015, RT-Thread Development Team
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License along
*  with this program; if not, write to the Free Software Foundation, Inc.,
*  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
* Change Logs:
* Date           Author       Notes
* 2009-01-05     Bernard      the first version
* 2015-08-01     xiaonong     the first version for stm32f7xx
* 2016-01-15     ArdaFu       the first version for stm32f4xx with STM32 HAL
* 2016-01-15     zyh          the first version for stm32f401rc with STM32 HAL
*/

#include "drv_usart.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx_hal.h"

/*  配置串口DMA*/
void uartslkDmaInit(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure;
	DMA_HandleTypeDef DMA_HandleTypeStruct;
	DMA_InitTypeDef DMA_InitStruct;
	
	__HAL_RCC_DMA1_CLK_ENABLE();
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	

	/* USART TX DMA 通道配置*/
	DMA_InitStruct.Channel = DMA_CHANNEL_4;
	DMA_InitStruct.Direction = DMA_PERIPH_TO_MEMORY;
	DMA_InitStruct.FIFOMode = DMA_FIFOMODE_DISABLE;
	DMA_InitStruct.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	DMA_InitStruct.MemBurst = DMA_MBURST_SINGLE;
	DMA_InitStruct.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.MemInc = DMA_MINC_ENABLE;
	DMA_InitStruct.Mode = DMA_NORMAL;
	DMA_InitStruct.PeriphBurst = DMA_PBURST_SINGLE;
	DMA_InitStruct.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA_InitStruct.PeriphInc = DMA_PINC_ENABLE;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	
	DMA_HandleTypeStruct.Init = DMA_InitStruct;
	DMA_HandleTypeStruct.Instance = DMA1_Stream6;
	DMA_HandleTypeStruct.StreamIndex = 6;
	DMA_HandleTypeStruct.StreamBaseAddress = DMA2_Stream6_BASE;
	
	HAL_DMA_Init(&DMA_HandleTypeStruct);
	
	

	
	
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UARTSLK_TYPE->DR;
//	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dmaBuffer;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_BufferSize = 0;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
//	DMA_InitStructure.DMA_Channel = UARTSLK_DMA_CH;

//	NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_DMA_IRQ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	isUartDmaInitialized = true;
}

/* STM32 uart driver */
struct drv_uart
{
    UART_HandleTypeDef UartHandle;
    IRQn_Type irq;
};

static rt_err_t drv_configure(struct rt_serial_device *serial,
struct serial_configure *cfg)
{
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct drv_uart *)serial->parent.user_data;

    uart->UartHandle.Init.BaudRate   = cfg->baud_rate;
    uart->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uart->UartHandle.Init.Mode       = UART_MODE_TX_RX;
    uart->UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    switch (cfg->data_bits)
    {
    case DATA_BITS_8:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case DATA_BITS_9:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
        break;
    case STOP_BITS_2:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_2;
        break;
    default:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
        break;
    }
    switch (cfg->parity)
    {
    case PARITY_NONE:
        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
        break;
    case PARITY_ODD:
        uart->UartHandle.Init.Parity     = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        uart->UartHandle.Init.Parity     = UART_PARITY_EVEN;
        break;
    default:
        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
        break;
    }

    if (HAL_UART_Init(&uart->UartHandle) != HAL_OK)
    {
        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t drv_control(struct rt_serial_device *serial,
int cmd, void *arg)
{
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->irq);
        /* disable interrupt */
        __HAL_UART_DISABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_EnableIRQ(uart->irq);
        /* enable interrupt */
        __HAL_UART_ENABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    }

    return RT_EOK;
}

static int drv_putc(struct rt_serial_device *serial, char c)
{
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;
    
    while((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_TXE) == RESET));
    uart->UartHandle.Instance->DR = c;
    return 1;
}

static int drv_getc(struct rt_serial_device *serial)
{
    int ch;
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;

    ch = -1;
    if (__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) 
        ch = uart->UartHandle.Instance->DR & 0xff;
    return ch;
}

static const struct rt_uart_ops drv_uart_ops =
{
    drv_configure,
    drv_control,
    drv_putc,
    drv_getc,
};
#if defined(RT_USING_UART1)
/* UART1 device driver structure */
static struct drv_uart uart1;
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart1;
    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART1 */
#if defined(RT_USING_UART2)
/* UART2 device driver structure */

static struct drv_uart uart2;
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart2;
    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART6)
/* UART2 device driver structure */

static struct drv_uart uart6;
struct rt_serial_device serial6;

void USART6_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart6;
    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial6, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART3 */

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		
    /**USART1 GPIO Configuration    
    PA15     ------> USART1_TX
    PB3     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(uartHandle->Instance==USART2)
  {
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		// 设置UART2中断
		HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		
		/* 串口接受数据寄存器非空中断*/
		__HAL_UART_ENABLE_IT(uartHandle, UART_IT_RXNE);
		
		rt_memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
		
		/* PA0 外部中断配置*/
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		/*  HAL库中没有EXTI的初始化结构体函数，将EXTI的初始化归纳到GPIO_InitTypeDef的Mode中*/
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		
		
		
		
		
		
		
		
		
		
		

		
		
  }
  else if(uartHandle->Instance==USART6)
  {
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA15    ------> USART1_TX
    PB3     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
  }
  else if(uartHandle->Instance==USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
  }
  else if(uartHandle->Instance==USART6)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
  }
} 

int hw_usart_init(void)
{
    struct drv_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
#ifdef RT_USING_UART1
    uart = &uart1;
    uart->UartHandle.Instance = USART1;
		uart->irq = USART1_IRQn;
    serial1.ops    = &drv_uart_ops;
    serial1.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
    RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
    uart);
#endif /* RT_USING_UART1 */
#ifdef RT_USING_UART2
    uart = &uart2;
    uart->UartHandle.Instance = USART2;
		uart->irq = USART2_IRQn;
    serial2.ops    = &drv_uart_ops;
    serial2.config = config;

    /* register UART2 device */
    rt_hw_serial_register(&serial2, "uart2",
    RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
    uart);
#endif /* RT_USING_UART2 */
#ifdef RT_USING_UART6
    uart = &uart6;
    uart->UartHandle.Instance = USART6;
		uart->irq = USART6_IRQn;
    serial6.ops    = &drv_uart_ops;
    serial6.config = config;

    /* register UART2 device */
    rt_hw_serial_register(&serial6, "uart6",
    RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
    uart);
#endif /* RT_USING_UART2 */

    return 0;
}
INIT_BOARD_EXPORT(hw_usart_init);
