#include <stdbool.h>
#include <string.h>
#include <rtthread.h>

#include "config_param.h"
#include "usblink.h"
#include "atkp.h"
#include "config_param.h"
#include "ledseq.h"
#include "pm.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"

///*FreeRTOS相关头文件*/
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

/*  USB通信驱动代码	 */

#define USBLINK_TX_QUEUE_SIZE 	30 /*接收队列个数*/

static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;

static bool isInit;
static atkp_t rxPacket;

static struct rt_messagequeue txMq;
atkp_t mqBuff[USBLINK_TX_QUEUE_SIZE];
//static xQueueHandle  txQueue;


/*usb连接初始化*/
void usblinkInit()
{
	if(isInit) return;
	
	usbd_cdc_vcp_Init();
	/*创建发送队列，USBLINK_TX_QUEUE_SIZE个消息*/
	rt_mq_init(&txMq, "txMq", &mqBuff, sizeof(atkp_t), sizeof(mqBuff), RT_IPC_FLAG_FIFO);
//	txQueue = xQueueCreate(USBLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
//	ASSERT(txQueue);
	isInit = true;
}

/*usb连接发送atkpPacket*/
bool usblinkSendPacket(const atkp_t *p)
{
	RT_ASSERT(p);
	RT_ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return rt_mq_send(&txMq, (void *)p, sizeof(*p));
	//ASSERT(p);
	//ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	//return xQueueSend(txQueue, p, 0);	
}



//USB发送ATKPPacket任务
void usblinkTxTask(void *param)
{
	atkp_t p;
	uint8_t sendBuffer[64];
	uint8_t cksum;
	uint8_t dataLen;
	while(1)
	{
		rt_mq_recv(&txMq, &p, sizeof(p), MAX_DELAY);
		//xQueueReceive(txQueue, &p, portMAX_DELAY);
		
		sendBuffer[0] = UP_BYTE1;
		sendBuffer[1] = UP_BYTE2;
		sendBuffer[2] = p.msgID;
		sendBuffer[3] = p.dataLen;
		memcpy(&sendBuffer[4], p.data, p.dataLen);
		cksum = 0;
		for (int i = 0; i < p.dataLen+4; i++)
		{
			cksum += sendBuffer[i];
		}
		dataLen = p.dataLen + 5;
		sendBuffer[dataLen - 1] = cksum;
		usbsendData(sendBuffer, dataLen);
		ledseqRun(DATA_TX_LED, seq_linkup);
	}
}

//USB虚拟串口接收ATKPPacket任务
void usblinkRxTask(void *param)
{
	uint8_t c;
	uint8_t dataIndex = 0;
	uint8_t cksum = 0;
	rxState = waitForStartByte1;
	while(1)
	{
		if (usbGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;
					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;
					break;
				case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						cksum += c;
					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{
						ledseqRun(DATA_RX_LED, seq_linkup);
						atkpReceivePacketBlocking(&rxPacket);
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;	
						//IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStartByte1;
					break;
				default:
					//ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStartByte1;
		}
	}
}

