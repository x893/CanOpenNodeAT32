#include "main.h"
#include "CO_app_AT32.h"

/**
  *  @brief  can gpio config
  *  @param  none
  *  @retval nones
  */
static void can_gpio_config(void)
{
	gpio_init_type gpio_init_struct;

	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	gpio_pin_remap_config(CAN1_GMUX_0010, TRUE);

	gpio_default_para_init(&gpio_init_struct);
	/* can tx pin */
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pins = GPIO_PINS_9;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOB, &gpio_init_struct);
	/* can rx pin */
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_8;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOB, &gpio_init_struct);
}

/**
  *  @brief  can configiguration.
  *  @param  none
  *  @retval none
  */
static void can_configuration(void)
{
	can_base_type can_base_struct;
	can_baudrate_type can_baudrate_struct;
	can_filter_init_type can_filter_init_struct;

	crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
	/* can base init */
	can_default_para_init(&can_base_struct);
	can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
	can_base_struct.ttc_enable = FALSE;
	can_base_struct.aebo_enable = TRUE;
	can_base_struct.aed_enable = TRUE;
	can_base_struct.prsf_enable = FALSE;
	can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
	can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;
	can_base_init(CAN1, &can_base_struct);

	/* can baudrate, set baudrate = pclk/(baudrate_div *(1 + bts1_size + bts2_size)) */
	can_baudrate_struct.baudrate_div = 6;
	can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
	can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
	can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
	can_baudrate_set(CAN1, &can_baudrate_struct);

	/* can filter init */
	can_filter_init_struct.filter_activate_enable = TRUE;
	can_filter_init_struct.filter_mode = CAN_FILTER_MODE_ID_MASK;
	can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
	can_filter_init_struct.filter_number = 0;
	can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
	can_filter_init_struct.filter_id_high = 0;
	can_filter_init_struct.filter_id_low = 0;
	can_filter_init_struct.filter_mask_high = 0;
	can_filter_init_struct.filter_mask_low = 0;
	can_filter_init(CAN1, &can_filter_init_struct);

	/* can interrupt config */
	nvic_irq_enable(CAN1_SE_IRQn, 0x00, 0x00);
	nvic_irq_enable(CAN1_RX0_IRQn, 0x00, 0x00);
	can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);

	/* error interrupt enable */
	can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);
	can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);
}

CANopenNodeAT32 CANOpenNode = {
	.HWInitFunction = can_configuration,
	.timerHandle = CAN_TIMER,
	.CANHandle = CAN_CAN,
	.desiredNodeID = 0x01,
	.baudrate = 250,
};

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
	system_clock_config();
	at32_board_init();

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	if ( canopen_storage_init( ) != CO_ERROR_NO )
		ErrorHandler( ERR_OD_INVALID );
#endif

	Timer_Init( );
	
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	can_gpio_config();

	canopen_app_init( &CANOpenNode );

	uint32_t timer = 0;
	uint32_t time_old, time_current, timeDifference_us;
    time_old = time_current = Timer_GetTick();

	while (1)
	{
		time_current = Timer_GetTick( );
		if ( time_current != time_old )
		{
			timeDifference_us = (time_current - time_old) * 1000;
			time_old = time_current;

			canopen_app_process( timeDifference_us );

#if (CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE
			if ( CANOpenNode.outStatusLEDGreen )
				at32_led_on(LED2);
			else
				at32_led_off(LED2);

			if ( CANOpenNode.outStatusLEDRed )
				at32_led_on(LED3);
			else
				at32_led_off(LED3);
#endif			
			if ( timer >= 1000 )
			{
				timer = 0;
				at32_led_off(LED4);
			}
			else if ( timer == 900 )
			{
				at32_led_on(LED4);
			}
			
			++timer;
		}
	}
}

void ErrorHandler( void )
{
	delay_ms(1000);
	NVIC_SystemReset( );
}
