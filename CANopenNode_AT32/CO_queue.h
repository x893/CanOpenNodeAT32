#ifndef __CO_QUEUE_H__
#define __CO_QUEUE_H__

#include <stdbool.h>
#include "main.h"
#include "CO_driver_target.h"

#ifdef __cplusplus
extern "C" {
#endif

	bool CO_TxQueuePut(CAN_TX_QUEUE_TYPE* packet);
	CAN_TX_QUEUE_TYPE* CO_TxQueueGet(void);
	void CO_TxQueueShift(void);

	bool CO_RxQueuePut(CAN_RX_QUEUE_TYPE* packet);
	CAN_RX_QUEUE_TYPE* CANRxQueueGet(void);
	void CANRxQueueShift(void);
	
#ifdef __cplusplus
}
#endif
#endif
