#include <string.h>
#include "CO_queue.h"

#define CAN_RX_QUEUE_SIZE   16 /* must be power of 2 */
#define CAN_TX_QUEUE_SIZE   32 /* must be power of 2 */

typedef struct { // CAN Rx queue
	uint16_t size;
	uint16_t head;
	CAN_RX_QUEUE_TYPE queue[CAN_RX_QUEUE_SIZE];
} CANRxQueue;

typedef struct { // CAN Tx queue
	uint16_t size;
	uint16_t head;
	CAN_TX_QUEUE_TYPE queue[CAN_TX_QUEUE_SIZE];
} CANTxQueue;

static CANRxQueue canRxQueue = {
  .size = 0,
  .head = 0
};

static CANTxQueue canTxQueue = {
  .size = 0,
  .head = 0
};

bool CO_TxQueuePut(CAN_TX_QUEUE_TYPE* packet)
{
	if (canTxQueue.size < CAN_TX_QUEUE_SIZE)
	{
		uint16_t putPos = (canTxQueue.head + canTxQueue.size) & (CAN_TX_QUEUE_SIZE - 1);
		memcpy((uint8_t*)&(canTxQueue.queue[putPos]), (uint8_t*)packet, sizeof(CAN_TX_QUEUE_TYPE));
		canTxQueue.size++;
		return true;
	}
	return false;
}

CAN_TX_QUEUE_TYPE* CO_TxQueueGet(void)
{
	if (canTxQueue.size == 0)
		return NULL;
	return &(canTxQueue.queue[canTxQueue.head]);
}

void CO_TxQueueShift(void)
{
	if (canTxQueue.size != 0)
	{
		canTxQueue.head = (canTxQueue.head + 1) & (CAN_TX_QUEUE_SIZE - 1);
		canTxQueue.size--;
	}
}

/*
 *
 */
bool CO_RxQueuePut(CAN_RX_QUEUE_TYPE* packet)
{
	if (canRxQueue.size < CAN_RX_QUEUE_SIZE)
	{
		uint16_t putPos = (canRxQueue.head + canRxQueue.size) & (CAN_RX_QUEUE_SIZE - 1);
		memcpy((uint8_t*)&(canRxQueue.queue[putPos]), (uint8_t*)packet, sizeof(CAN_RX_QUEUE_TYPE));
		canRxQueue.size++;
		return true;
	}
	return false;
}

CAN_RX_QUEUE_TYPE* CANRxQueueGet(void)
{
	if (canRxQueue.size == 0)
		return NULL;
	return &(canRxQueue.queue[canRxQueue.head]);
}

void CANRxQueueShift(void)
{
	if (canRxQueue.size != 0)
	{
		canRxQueue.head = (canRxQueue.head + 1) & (CAN_RX_QUEUE_SIZE - 1);
		canRxQueue.size--;
	}
}
