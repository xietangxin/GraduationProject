#include <rtthread.h>
#include <string.h>
#include "radiolink.h"
#include "config_param.h"
#include "drv_led.h"
#include "ledseq.h"

//#include "uart_syslink.h"

/*FreeRtos includes*/
//#include "FreeRTOS.h"
//#include "task.h"
//#include "semphr.h"
//#include "queue.h"

/* ����ͨ����������	*/

#define RADIOLINK_TX_QUEUE_SIZE  30 /*���ն��и���*/

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
static atkp_t txPacket;
static atkp_t rxPacket;
//static xQueueHandle  txQueue;
static rt_messagequeue mq;
char mq_buf[sizeof(atkp_t) * RADIOLINK_TX_QUEUE_SIZE];

static void atkpPacketDispatch(atkp_t *rxPacket);

//radiolink����ATKPPacket����
void radiolinkTask(void *param)
{
	rxState = waitForStartByte1;
	
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;

	while(1)
	{
		if (uartslkGetDataWithTimout(&c))
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
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,���ݳ���Ϊ0��У��1*/
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
					if (cksum == c)	/*����У����ȷ*/
					{
						atkpPacketDispatch(&rxPacket);
					} 
					else	/*У�����*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStartByte1;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*��ʱ����*/
		{
			rxState = waitForStartByte1;
		}
	}
}


void radiolinkInit(void)
{
	if (isInit) return;
	uartslkInit();
	
	/*�������Ͷ��У�CRTP_TX_QUEUE_SIZE����Ϣ*/
	rt_mq_init(&mq, "radiolinkmq", mq_buf, sizeof(atkp_t), sizeof(mq_buf), RT_IPC_FLAG_FIFO);
	//txQueue = xQueueCreate(RADIOLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	RT_ASSERT(mq != RT_NULL);
	//ASSERT(txQueue); 
	
	isInit = true;
}

/*���ATKPPacket����ͨ������DMA����*/
static void uartSendPacket(atkp_t *p)
{
	int dataSize;
	u8 cksum = 0;
	u8 sendBuffer[36];
	
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);

	sendBuffer[0] = UP_BYTE1;
	sendBuffer[1] = UP_BYTE2;
	sendBuffer[2] = p->msgID;
	sendBuffer[3] = p->dataLen;
	
	memcpy(&sendBuffer[4], p->data, p->dataLen);
	dataSize = p->dataLen + 5;//����cksum
	/*����У���*/
	for (int i=0; i<dataSize-1; i++)
	{
		cksum += sendBuffer[i];
	}
	sendBuffer[dataSize-1] = cksum;
	
	/*����DMA����*/
	uartslkSendDataDmaBlocking(dataSize, sendBuffer);
}

/*radiolink���յ�ATKPPacketԤ����*/
static void atkpPacketDispatch(atkp_t *rxPacket)
{
	atkpReceivePacketBlocking(rxPacket);
	
	if( rxPacket->msgID == DOWN_POWER)
	{;}/*do noting*/
	else
	{
		//ledseqRun(DATA_RX_LED, seq_linkup); ????????
		/*���յ�һ��ң���������ݰ�����һ����*/
		if (rt_mq_recv(&mq, &txPacket, sizeof(txPacket), 0) == RT_EOK)
	//	if(xQueueReceive(txQueue, &txPacket, 0) == pdTRUE)
		{
			ASSERT(txPacket.dataLen <= ATKP_MAX_DATA_SIZE);
			//ledseqRun(DATA_TX_LED, seq_linkup); ?????????????
			uartSendPacket(&txPacket);
		}
	}
}

// ?????????????????    rt_thread ����������packet
bool radiolinkSendPacket(const atkp_t *p)
{
	RT_ASSERT(p != RT_NULL);
	RT_ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return rt_mq_send(&mq, p, sizeof(*p));
	//return xQueueSend(txQueue, p, 0);
}

bool radiolinkSendPacketBlocking(const atkp_t *p)
{
	RT_ASSERT(p);
	RT_ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return rt_mq_send(&mq, p, sizeof(*p))
	//return xQueueSend(txQueue, p, portMAX_DELAY);	
}

//��ȡʣ�����txQueue����
int radiolinkGetFreeTxQueuePackets(void)	
{
	//???????????????????????
	//return (RADIOLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}
