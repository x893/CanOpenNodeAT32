#include "main.h"
#include "CO_app_AT32.h"
#include "CO_Flash.h"

#define LED_RED		LED2
#define LED_GREEN	LED4

void can_configuration( uint16_t baudrate );

CANopenNodeAT32 CANOpenNode = {
	.HWInitFunction = can_configuration,
	.timerHandle = CAN_TIMER,
	.CANHandle = CAN_CAN,
	.desiredNodeID = 0x01,
	.baudrate = 250,
};

void usb_init( void );
void usb_process( void );

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
	system_clock_config();
	at32_board_init();

	__enable_irq();
	HardwareInit( );

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	if ( CO_Storage_Init( ) != CO_ERROR_NO )
		ErrorHandler( );
#endif

	usb_init( );

	canopen_app_init( &CANOpenNode );

	uint32_t time_old, time_current, timeDifference_us;
    time_old = time_current = Timer_GetTick();

	while (1)
	{
		usb_process( );

		time_current = Timer_GetTick( );
		if ( time_current != time_old )
		{
			timeDifference_us = (time_current - time_old) * 1000;
			time_old = time_current;

			canopen_app_process( timeDifference_us );

#if (CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE

			if ( CO_LED_GREEN( CANOpenNode.canOpenStack->LEDs, CO_LED_CANopen ) )
				at32_led_on(LED_GREEN);
			else
				at32_led_off(LED_GREEN);

			if ( CO_LED_RED( CANOpenNode.canOpenStack->LEDs, CO_LED_CANopen ) )
				at32_led_on(LED_RED);
			else
				at32_led_off(LED_RED);
#endif
		}
	}
}

void ErrorHandler( void )
{
	delay_ms(1000);
	NVIC_SystemReset( );
}
