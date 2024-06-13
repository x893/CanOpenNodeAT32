#include <string.h>
#include "at32f415_board.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

otg_core_type otg_core_struct;

extern linecoding_type linecoding;

#define  USB_BUFFER_SIZE  256
uint8_t usb_buffer[USB_BUFFER_SIZE];
uint8_t usb_tx_buffer[USB_BUFFER_SIZE];

typedef struct {
	uint16_t write_index;
	uint16_t read_index;
	uint16_t overflow_cnt;
	uint8_t send_zero_packet;
} usb_context_t;
static usb_context_t usb_context;

void usb_write( uint8_t* src, uint16_t length )
{
	register usb_context_t* ctx = &usb_context;
	register uint16_t index = ctx->write_index;
	register uint16_t next = ctx->write_index;
	while ( length-- != 0 )
	{
		next = index + 1;
		if ( next >= USB_BUFFER_SIZE )
			next = 0;
		if ( next == ctx->read_index )
			break;
		usb_tx_buffer[index] = *src++;
		index = next;
	}
	ctx->write_index = index;
}

/**
  * @brief  this function handles usart2 receive data.
  * @param  none
  * @retval the data len
  */
static uint16_t usb_get_tx_count(void)
{
	register usb_context_t* ctx = &usb_context;

	if ( ctx->read_index == ctx->write_index )
		return 0;

	uint16_t data_len;
	if ( ctx->write_index > ctx->read_index )
		data_len = ctx->write_index - ctx->read_index;
	else if ( ctx->write_index == 0 && ctx->write_index != ctx->read_index )
		data_len = USB_BUFFER_SIZE - ctx->read_index;
	else
		data_len = (USB_BUFFER_SIZE - 1) + ctx->write_index - ctx->read_index;
	return data_len;
}

void usb_process( )
{
	register usb_context_t* ctx = &usb_context;
	uint16_t timeout;
	uint16_t data_len;

    /* get usb vcp receive data */
	if ( (data_len = usb_vcp_get_rxdata( &otg_core_struct.dev, usb_buffer )) != 0 )
	{
		// Procccessing data (simple echo)
		usb_write( usb_buffer, data_len );
	}

	if ( (data_len = usb_get_tx_count( )) != 0 || ctx->send_zero_packet == 1 )
	{
		ctx->send_zero_packet = ( data_len != 0 ) ? 1 : 0;

		if ( (ctx->read_index + data_len) < USB_BUFFER_SIZE )
		{
			timeout = 50000;
			do
			{
				/* send data to host */
				memcpy( usb_buffer, &usb_tx_buffer[ctx->read_index], data_len );
				if ( usb_vcp_send_data(&otg_core_struct.dev, usb_buffer, data_len) == SUCCESS )
				{
					ctx->read_index = ctx->read_index + data_len;
					break;
				}
			} while ( timeout-- );
		}
		/* process the fifo overflow */
		else
		{
			timeout = 50000;
			do
			{
				/* send data to host */
				memcpy( usb_buffer, &usb_tx_buffer[ctx->read_index], USB_BUFFER_SIZE - ctx->read_index );
				if ( usb_vcp_send_data(&otg_core_struct.dev, usb_buffer, USB_BUFFER_SIZE - ctx->read_index) == SUCCESS )
				{
					/* get fifo overflow data count */
					ctx->overflow_cnt = data_len - (USB_BUFFER_SIZE - ctx->read_index);
					ctx->read_index = 0;
					break;
				}
			} while ( timeout-- );

			timeout = 50000;
			do
			{
				/* send data to host */
				memcpy( usb_buffer, &usb_tx_buffer[ctx->read_index], ctx->overflow_cnt );
				if ( usb_vcp_send_data( &otg_core_struct.dev, usb_buffer, ctx->overflow_cnt) == SUCCESS )
				{
					ctx->read_index = ctx->overflow_cnt;
					break;
				}
			} while ( timeout-- );
		}
	}
}

/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
    switch(system_core_clock)
    {
      /* 48MHz */
      case 48000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1);
        break;

      /* 72MHz */
      case 72000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1_5);
        break;

      /* 96MHz */
      case 96000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2);
        break;

      /* 120MHz */
      case 120000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2_5);
        break;

      /* 144MHz */
      case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;

      default:
        break;
    }
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
{
	gpio_init_type gpio_init_struct;

	crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
	gpio_default_para_init(&gpio_init_struct);

	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

#ifdef USB_SOF_OUTPUT_ENABLE
	crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
	gpio_init_struct.gpio_pins = OTG_PIN_SOF;
	gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
#endif

	/* otgfs use vbus pin */
#ifndef USB_VBUS_IGNORE
	gpio_init_struct.gpio_pins = OTG_PIN_VBUS;
	gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init(OTG_PIN_GPIO, &gpio_init_struct);
#endif

}
#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  usb low power wakeup interrupt config
  * @param  none
  * @retval none
  */
void usb_low_power_wakeup_config(void)
{
  exint_init_type exint_init_struct;

  exint_default_para_init(&exint_init_struct);

  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = OTG_WKUP_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  nvic_irq_enable(OTG_WKUP_IRQ, 3, 0);
}

/**
  * @brief  this function handles otgfs wakup interrupt.
  * @param  none
  * @retval none
  */
void OTG_WKUP_HANDLER(void)
{
  exint_flag_clear(OTG_WKUP_EXINT_LINE);
}

#endif

#ifdef USB_VIRTUAL_COMPORT
/**
  *
  */
void usb_usart_config( linecoding_type linecoding)
{
#if 0
	usart_stop_bit_num_type usart_stop_bit;
	usart_data_bit_num_type usart_data_bit;
	usart_parity_selection_type usart_parity_select;

	/* stop bit */
	switch(linecoding.format)
	{
	case 0x0:
		usart_stop_bit = USART_STOP_1_BIT;
		break;
	/* to be used when transmitting and receiving data in smartcard mode */
    case 0x1:
		usart_stop_bit = USART_STOP_1_5_BIT;
		break;
	case 0x2:
		usart_stop_bit = USART_STOP_2_BIT;
		break;
	default :
		break;
	}

	/* parity */
	switch(linecoding.parity)
	{
	case 0x0:
		usart_parity_select = USART_PARITY_NONE;
		break;
	case 0x1:
		usart_parity_select = USART_PARITY_ODD;
		break;
	case 0x2:
		usart_parity_select = USART_PARITY_EVEN;
		break;
	/* hardware usart not support partiy for mark and space */
	case 0x3:
	case 0x4:
		break;
	default :
		break;
	}

	if ( USART_PARITY_NONE == usart_parity_select )
	{
		/* data bits */
		switch(linecoding.data)
		{
		/* hardware usart not support data bits for 5/6 */
		case 0x5:
		case 0x6:
		case 0x7:
			break;
		case 0x8:
			usart_data_bit = USART_DATA_8BITS;
			break;
		/* hardware usart not support data bits for 16 */
		case 0x10:
			break;
		default :
			break;
		}
	}
	else
	{
		/* data bits */
		switch(linecoding.data)
		{
		/* hardware usart not support data bits for 5/6 */
		case 0x5:
		case 0x6:
			break;
		case 0x7:
			usart_data_bit = USART_DATA_8BITS;
			break;
		case 0x8:
			usart_data_bit = USART_DATA_9BITS;
			break;
		/* hardware usart not support data bits for 16 */
		case 0x10:
			break;
		default :
			break;
		}
	}
#endif
}
#endif

/**
  *
  */
void usb_init( )
{
	usb_usart_config(linecoding);
	usb_gpio_config( );
#ifdef USB_LOW_POWER_WAKUP
	usb_low_power_wakeup_config();
#endif
	
	/* enable otgfs clock */
	crm_periph_clock_enable(OTG_CLOCK, TRUE);

	/* select usb 48m clcok source */
	usb_clock48m_select(USB_CLK_HEXT);

	/* enable otgfs irq */
	nvic_irq_enable(OTG_IRQ, 3, 0);

	/* init usb */
	usbd_init ( &otg_core_struct,
				USB_FULL_SPEED_CORE_ID,
				USB_ID,
				&cdc_class_handler,
				&cdc_desc_handler
	);
}

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
	usbd_irq_handler(&otg_core_struct);
}

/**
  * @brief  usb delay millisecond function.
  * @param  ms: number of millisecond delay
  * @retval none
  */
void usb_delay_ms(uint32_t ms)
{
	/* user can define self delay function */
	delay_ms(ms);
}

/**
  * @brief  usb delay microsecond function.
  * @param  us: number of microsecond delay
  * @retval none
  */
void usb_delay_us(uint32_t us)
{
  delay_us(us);
}
