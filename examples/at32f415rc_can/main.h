#ifndef __MAIN_H
#define __MAIN_H

#include "at32f415_board.h"
#include "at32f415_clock.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CAN					CAN1

#define CAN_TIMER				TMR3
#define CAN_TIMER_IRQ			TMR3_GLOBAL_IRQn
#define CAN_TIMER_IRQHandler	TMR3_GLOBAL_IRQHandler
#define CAN_TIMER_CLOCK			CRM_TMR3_PERIPH_CLOCK

// #define CAN_SE_IRQn		CAN1_SE_IRQn
#define CAN_RX0_IRQn		CAN1_RX0_IRQn
#define CAN_RX1_IRQn		CAN1_RX1_IRQn
// #define CAN_TX_IRQn		CAN1_TX_IRQn

void ErrorHandler( void );

#ifdef __cplusplus
}
#endif

#endif
