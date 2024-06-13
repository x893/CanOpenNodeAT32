#ifndef CANOPENAT32_CO_APP_AT32_H_
#define CANOPENAT32_CO_APP_AT32_H_

#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	/* This is the Node ID that you ask the CANOpen stack to assign to your device,
	 * although it might not always be the final NodeID, after calling canopen_app_init()
	 * you should check ActiveNodeID of CANopenNodeAT32 structure for assigned Node ID.
	 */
    uint8_t		desiredNodeID;

	/* Assigned Node ID */
    uint8_t		activeNodeID;
	/* This is the baudrate you've set in your CubeMX Configuration */
    uint16_t	baudrate;
	
	/* Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
	 * please note that CANOpenAT32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function in your codes, please take required steps
	 */
    tmr_type*	timerHandle;

    /* Pass in the CAN Handle to this function and it wil be used for all CAN Communications. */
	can_type* CANHandle;

    void (*HWInitFunction)(uint16_t baudrate); /* Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init */

    CO_t* canOpenStack;

} CANopenNodeAT32;


// In order to use CANOpenAT32, you'll have it have a canopenNodeAT32 structure somewhere in your codes, it is usually residing in CO_app_AT32.c
extern CANopenNodeAT32* pCANopenNodeAT32;


/* This function will initialize the required CANOpen Stack objects, allocate the memory and prepare stack for communication reset*/
int canopen_app_init( CANopenNodeAT32* canopenAT32 );

/* This function will check the input buffers and any outstanding tasks that are not time critical, this function should be called regurarly from your code (i.e from your while(1))*/
void canopen_app_process( uint32_t timeDifference_us );

void Timer_Init( void );
uint32_t Timer_GetTick( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANOPENAT32_CO_APP_AT32_H_ */
