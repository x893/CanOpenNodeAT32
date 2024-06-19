#include <string.h>
#include "main.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

#include "CO_queue.h"

#include "CANopen.h"
#include "OD.h"

otg_core_type otg_core_struct;

extern linecoding_type linecoding;

#define  USB_BUFFER_SIZE  256
uint8_t usb_buffer[USB_BUFFER_SIZE];
uint8_t usb_tx_buffer[USB_BUFFER_SIZE];

typedef struct {
	uint16_t	write_index;
	uint16_t	read_index;
	uint16_t	overflow_cnt;
	uint8_t		send_zero_packet;
	bool		log_enable;
} usb_context_t;

static usb_context_t usb_context = { 0 };

void usb_send(uint8_t ch)
{
	register usb_context_t* ctx = &usb_context;
	register uint16_t index = ctx->write_index;
	register uint16_t next = ctx->write_index + 1;
	if ( next >= USB_BUFFER_SIZE )
		next = 0;
	if ( next != ctx->read_index )
	{
		usb_tx_buffer[index] = ch;
		ctx->write_index = next;
	}
}

void usb_send_string(uint8_t* src, register usb_context_t* ctx)
{
	register uint16_t index = ctx->write_index;
	register uint16_t next = ctx->write_index;
	while ( *src != 0 )
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
static uint16_t usb_get_tx_count(register usb_context_t* ctx)
{
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

static uint32_t hex2binN4(uint8_t ch)
{
	ch -= '0';
	if (ch > 9)
	{
		ch -= ('A' - '0');
		if (ch >= 6)
		{
			ch -= ('a' - 'A');
			if (ch >= 6)
				return 0x1000;
		}
		ch += 0x0A;
	}
	return ch;
}

static void convertUSBtin2AT32(uint8_t* package)
{
	CAN_RX_QUEUE_TYPE packetRx;
	CAN_TX_QUEUE_TYPE packetTx;

	uint32_t data;
	uint8_t idx = 0;

	for (;; )
	{
		if (package[0] != 't') break;

		packetRx.id_type =
			packetTx.id_type = CAN_ID_STANDARD;

		data = (hex2binN4(package[1]) << 8);	// 0x100x..
		data |= (hex2binN4(package[2]) << 4);	//  0x100x.
		data |= (hex2binN4(package[3]));		//   0x100x
		if (data & 0xFFF000) break;				// 0x111xxx
		packetRx.standard_id =
			packetTx.standard_id = data;

		data = hex2binN4(package[4]);
		if (data & 0xFFF000) break;
		data &= 0x0F;
		packetRx.dlc =
			packetTx.dlc = data;

		package += 5;

		while (idx != packetRx.dlc)
		{
			data = (hex2binN4(*package++) << 4);
			data |= hex2binN4(*package++);
			if (data & 0xFFF000) break;

			packetRx.data[idx] =
				packetTx.data[idx] = data;
			idx++;
		}
		if (data & 0xFFF000) break;
		if (*package != '\0') break;

		packetRx.filter_index = 0;
		packetRx.frame_type =
			packetTx.frame_type = CAN_TFT_USB;

		CO_RxQueuePut(&packetRx);
		CO_TxQueuePut(&packetTx);

		break;
	}
}

static char hex2char(uint32_t value)
{
	value &= 0x0F;
	if (value > 9)
		return value + ('A' - 0x0A);
	return value + '0';
}

static uint8_t* hex8string(uint8_t* dst, uint32_t value)
{
	*dst++ = hex2char(value >> 28);
	*dst++ = hex2char(value >> 24);
	*dst++ = hex2char(value >> 20);
	*dst++ = hex2char(value >> 16);
	*dst++ = hex2char(value >> 12);
	*dst++ = hex2char(value >> 8);
	*dst++ = hex2char(value >> 4);
	*dst++ = hex2char(value >> 0);
	return dst;
}

static uint8_t* hex3string(uint8_t* dst, uint32_t value)
{
	*dst++ = hex2char(value >> 8);
	*dst++ = hex2char(value >> 4);
	*dst++ = hex2char(value >> 0);
	return dst;
}

static void enterSystemBoot( )
{
	// Set flags to enter bootloader
	uint32_t* boot = (uint32_t*)SRAM_BASE;
	boot[0] = 0xDEADBEEF;
	boot[1] = 0xCAFEBEEF;
	NVIC_SystemReset();
}

static void process_command(uint8_t* command, register usb_context_t* ctx)
{
	switch (*command)
	{
	case 't': // STANDARD TX
	{
		convertUSBtin2AT32(command);
		usb_send('z');
		break;
	}
	case 'C':
		ctx->log_enable = false;
		break;
	case 'O':
		ctx->log_enable = true;
		break;
	case 'X':
		if (command[1] == '\0')
			enterSystemBoot( );
		break;
	case 'N':	// serialNumber
		hex8string(command + 1, OD_PERSIST_COMM.x1018_identity.serialNumber);
		command[9] = '\0';
		usb_send_string(command, ctx);
		break;
	case 'v':	// firmwareVersion
		hex8string(command + 1, OD_PERSIST_COMM.x1018_identity.revisionNumber);
		command[9] = '\0';
		usb_send_string(command, ctx);
		break;
	case 'V':	// hardwareVersion
		hex8string(command + 1, OD_PERSIST_COMM.x1018_identity.vendor_ID);
		command[9] = '\0';
		usb_send_string(command, ctx);
		break;
	case 'S':	// Speed: 0: 10000, 1: 20000, 2: 50000, 3: 100000, 4: 125000, 5: 250000, 6: 500000, 7: 800000, 8: 1000000
		break;
	case 's':	// Custom speed: xxXXxx
	case 'L':	// LISTENONLY
	case 'l':	// LOOPBACK
	case 'r':	// STANDARD RTR
		// r 001 8
		//   id  length
	case 'R':	// EXTENDED RTR
		// r 00000001 8
		//   id       length
	case 'T': // EXTENDED TX
		// T 00000001 0
		//   id       length
		// T 00000001 8      1122334455667788
		//   id       length data
	case 'W':
		break;
	default:
		return;
	}
	usb_send('\r');
}

void SendTxToUSB(CAN_TX_QUEUE_TYPE* msg)
{
	register usb_context_t* ctx = &usb_context;

	if ( ! ctx->log_enable)
		return;

	uint8_t buffer[24];
	uint8_t* dst = buffer;
	uint8_t data_length = 0;
	uint8_t* src = msg->data;

	if (msg->id_type == CAN_ID_STANDARD)
	{
		if (msg->frame_type == CAN_TFT_DATA)
		{
			*dst++ = 't';
			data_length = msg->dlc;
		}
		else if (msg->frame_type == CAN_TFT_REMOTE)
			*dst++ = 'r';
		else
			return;
		dst = hex3string(dst, msg->standard_id);
	}
	else if (msg->id_type == CAN_ID_EXTENDED)
	{
		if (msg->frame_type == CAN_TFT_DATA)
		{
			*dst++ = 'T';
			data_length = msg->dlc;
		}
		else if (msg->frame_type == CAN_TFT_REMOTE)
			*dst++ = 'R';
		else
			return;
		dst = hex8string(dst, msg->extended_id);
	}
	else
		return;

	*dst++ = hex2char(msg->dlc);
	while (data_length-- != 0)
	{
		*dst++ = hex2char(*src >> 4);
		*dst++ = hex2char(*src++ >> 0);
	}
	*dst++ = '\r';
	*dst++ = '\0';
	usb_send_string(buffer, ctx);
}

void SendRxToUSB( CAN_RX_QUEUE_TYPE *msg )
{
	register usb_context_t* ctx = &usb_context;

	if ( ! ctx->log_enable )
		return;
	
	uint8_t buffer[24];
	uint8_t *dst = buffer;
	uint8_t data_length = 0;
	uint8_t *src = msg->data;

	if ( msg->id_type == CAN_ID_STANDARD )
	{
		if ( msg->frame_type == CAN_TFT_DATA )
		{
			dst[0] = 't';
			data_length = msg->dlc;
		}
		else if ( msg->frame_type == CAN_TFT_REMOTE )
			dst[0] = 'r';
		else
			return;

		dst = hex3string( &dst[1], msg->standard_id );
	}
	else if ( msg->id_type == CAN_ID_EXTENDED )
	{
		if ( msg->frame_type == CAN_TFT_DATA )
		{
			dst[0] = 'T';
			data_length = msg->dlc;
		}
		else if ( msg->frame_type == CAN_TFT_REMOTE )
			dst[0] = 'R';
		else
			return;

		dst = hex8string( &dst[1], msg->extended_id );
	}
	else
		return;

	*dst++ = hex2char(msg->dlc);

	while ( data_length-- != 0 )
	{
		*dst++ = hex2char(*src >> 4);
		*dst++ = hex2char(*src++ >> 0);
	}
	*dst++ = '\r';
	*dst = '\0';
	usb_send_string( buffer, ctx );
}

void usb_process( )
{
	static uint8_t usb_command[64];
	static uint16_t usb_command_idx = 0;

	register usb_context_t* ctx = &usb_context;
	uint16_t timeout;
	uint16_t data_len;
	uint8_t* src = usb_buffer;

    /* get usb vcp receive data */
	if ( (data_len = usb_vcp_get_rxdata( &otg_core_struct.dev, usb_buffer )) != 0 )
	{
		while (data_len-- != 0)
		{
			if (*src == '\r')
			{
				if (usb_command_idx != 0 && usb_command_idx < (sizeof(usb_command) - 1))
				{
					usb_command[usb_command_idx] = '\0';
					process_command(usb_command, ctx);
				}
				usb_command_idx = 0;
			}
			else if (usb_command_idx < (sizeof(usb_command) - 1))
				usb_command[usb_command_idx++] = *src;
			++src;
		}
	}

	if ( (data_len = usb_get_tx_count(ctx)) != 0 || ctx->send_zero_packet == 1 )
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
