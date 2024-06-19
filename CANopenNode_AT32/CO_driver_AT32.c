#include "301/CO_driver.h"
#include "CO_app_AT32.h"
#include "CO_queue.h"

#define CAN_RX_PIN			GPIO_PINS_8
#define CAN_RX_PORT			GPIOB
#define CAN_TX_PIN			GPIO_PINS_9
#define CAN_TX_PORT			GPIOB
#define CAN_GPIO_CLOCK		CRM_GPIOB_PERIPH_CLOCK
#define CAN_GPIO_MUX		CAN1_GMUX_0010

#define CAN_CLOCK			CRM_CAN1_PERIPH_CLOCK

//--- #define CAN_SE_IRQn			CAN1_SE_IRQn
//--- #define CAN_TX_IRQn			CAN1_TX_IRQn
#define CAN_RX0_IRQn		CAN1_RX0_IRQn
#define CAN_RX1_IRQn		CAN1_RX1_IRQn

#define CAN_TX_IRQHandler	CAN1_TX_IRQHandler
#define CAN_RX0_IRQHandler	CAN1_RX0_IRQHandler
#define CAN_RX1_IRQHandler	CAN1_RX1_IRQHandler
#define CAN_SE_IRQHandler	CAN1_SE_IRQHandler

typedef struct {
	uint16_t bitrate;
	can_baudrate_type can_baudrate;
} CO_CANbitRateData_t;

static const CO_CANbitRateData_t CO_CANbitRateData[] = {
	/* Index 0 - default baudrate */
	{	.bitrate = 250,
		.can_baudrate = {
			.baudrate_div = 6,
			.rsaw_size = CAN_RSAW_3TQ,
			.bts1_size = CAN_BTS1_8TQ,
			.bts2_size = CAN_BTS2_3TQ,
			}
	},{	.bitrate = 125,
		.can_baudrate = {
			.baudrate_div = 36,
			.rsaw_size = CAN_RSAW_2TQ,
			.bts1_size = CAN_BTS1_13TQ,
			.bts2_size = CAN_BTS2_2TQ
		}
	},{	.bitrate = 500,
		.can_baudrate = {
			.baudrate_div = 9,
			.rsaw_size = CAN_RSAW_2TQ,
			.bts1_size = CAN_BTS1_13TQ,
			.bts2_size = CAN_BTS2_2TQ
		}
	},{	.bitrate = 1000,
		.can_baudrate = {
			.baudrate_div = 9,
			.rsaw_size = CAN_RSAW_2TQ,
			.bts1_size = CAN_BTS1_5TQ,
			.bts2_size = CAN_BTS2_2TQ
		}
	}
};

/**
  * @brief			Get CAN bitrate parameters table index
  * @param bitRate	Bitrate in kbps
  * @return			Index (0...) or -1
  */
int can_get_bitrate_index(uint16_t bitRate)
{
	if ( bitRate != 0 )
	{
    for (
		size_t i = 0;
		i < (sizeof(CO_CANbitRateData) / sizeof(CO_CANbitRateData[0]));
		i++
	)
		if (CO_CANbitRateData[i].bitrate == bitRate)
            return i;
	}
    return -1;
}

/**
  *  @brief  can gpio config
  *  @param  none
  *  @retval nones
  */
static void can_gpio_config(void)
{
	gpio_init_type gpio_init_struct;

	crm_periph_clock_enable(CAN_GPIO_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	gpio_pin_remap_config(CAN_GPIO_MUX, TRUE);

	gpio_default_para_init(&gpio_init_struct);

	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_pins = CAN_TX_PIN;
	gpio_init(CAN_TX_PORT, &gpio_init_struct);
	
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init_struct.gpio_pins = CAN_RX_PIN;
	gpio_init(CAN_RX_PORT, &gpio_init_struct);
}

/**
  *  @brief  can configiguration.
  *  @param  baudrate in Kbps or 0 for default
  *  @retval none
  */
void can_configuration( uint16_t baudrate )
{
	can_base_type can_base_struct;
	can_filter_init_type can_filter_init_struct;

	can_gpio_config( );

	crm_periph_clock_enable(CAN_CLOCK, TRUE);
	/* can base init */
	can_default_para_init(&can_base_struct);
	can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
	can_base_struct.ttc_enable = FALSE;
	can_base_struct.aebo_enable = TRUE;
	can_base_struct.aed_enable = TRUE;
	can_base_struct.prsf_enable = FALSE;
	can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
	can_base_struct.mmssr_selection = CAN_SENDING_BY_ID;
	if ( can_base_init(CAN_CAN, &can_base_struct) == SUCCESS )
	{
		int index = can_get_bitrate_index( baudrate );
		if (index < 0)
			index = 0;
		if (can_baudrate_set(
				CAN_CAN,
				(can_baudrate_type*) &CO_CANbitRateData[index].can_baudrate
				) == SUCCESS)
		{
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
			can_filter_init(CAN_CAN, &can_filter_init_struct);

			/* can interrupt config */
#ifdef CAN_TX_IRQn
			nvic_irq_enable(CAN_TX_IRQn,  4, 0);
			can_interrupt_enable(CAN_CAN, CAN_TCIEN_INT, TRUE);
#endif
#ifdef CAN_RX0_IRQn
			nvic_irq_enable(CAN_RX0_IRQn, 4, 0);
			can_interrupt_enable(CAN_CAN, CAN_RF0MIEN_INT, TRUE);
#endif
#ifdef CAN_RX1_IRQn
			nvic_irq_enable(CAN_RX1_IRQn, 4, 0);
			can_interrupt_enable(CAN_CAN, CAN_RF1MIEN_INT, TRUE);
#endif
#ifdef CAN_SE_IRQn
			/* error interrupt enable */
			nvic_irq_enable(CAN_SE_IRQn,  4, 0);
			can_interrupt_enable(CAN_CAN, CAN_ETRIEN_INT, TRUE);
			can_interrupt_enable(CAN_CAN, CAN_EOIEN_INT, TRUE);
#endif
		}
	}
}

/**
  *
  *
  */
static volatile uint32_t vuTicks;
uint32_t Timer_GetTick( )
{
	return vuTicks;
}

void CAN_TIMER_IRQHandler( )
{
	if ( tmr_interrupt_flag_get(CAN_TIMER, TMR_OVF_FLAG) != RESET )
	{
		tmr_flag_clear(CAN_TIMER, TMR_OVF_FLAG);
		++vuTicks;
    }
}

void HardwareInit( )
{
	crm_clocks_freq_type crm_clocks_freq_struct = {0};

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	debug_periph_mode_set(0
		| DEBUG_SLEEP
		| DEBUG_DEEPSLEEP
		| DEBUG_STANDBY
		| DEBUG_WDT_PAUSE
		| DEBUG_WWDT_PAUSE
		| DEBUG_TMR1_PAUSE
		| DEBUG_TMR2_PAUSE
		| DEBUG_CAN1_PAUSE
		, TRUE
	);

	/* get system clock */
	crm_clocks_freq_get(&crm_clocks_freq_struct);

	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CAN_TIMER_CLOCK, TRUE);

	gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);

	tmr_reset(CAN_TIMER);

	tmr_base_init(CAN_TIMER, 10 - 1, (crm_clocks_freq_struct.ahb_freq / 10000) - 1);
	tmr_cnt_dir_set(CAN_TIMER, TMR_COUNT_UP);
	tmr_clock_source_div_set( CAN_TIMER, TMR_CLOCK_DIV1 );
	tmr_flag_clear( CAN_TIMER, TMR_OVF_FLAG );

	/* overflow interrupt enable */
	tmr_interrupt_enable(CAN_TIMER, TMR_OVF_INT, TRUE);
	nvic_irq_enable(CAN_TIMER_IRQ, 1, 0);
}

/* Local instance of global CAN module */
static CO_CANmodule_t* CANModule_local = NULL;

/* CAN masks for identifiers */
#define CANID_MASK		0x07FF	/*!< CAN standard ID mask */
#define FLAG_RTR		0x8000	/*!< RTR flag, part of identifier */

/******************************************************************************/
void
CO_CANsetConfigurationMode(void* CANptr)
{
    /* Put CAN module in configuration mode */
    if ( CANptr != NULL &&
		((CANopenNodeAT32*) CANptr)->CANHandle != NULL
	)
	{
		can_reset( ((CANopenNodeAT32*)CANptr)->CANHandle );
	}
}

/******************************************************************************/
void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule)
{
    /* Put CAN module in normal mode */
    if ( CANmodule->CANptr != NULL )
	{
		can_operating_mode_set(
			((CANopenNodeAT32*)CANmodule->CANptr)->CANHandle,
			CAN_OPERATINGMODE_COMMUNICATE
		);
		CANmodule->CANnormal = true;
	}
}

/******************************************************************************/
CO_ReturnError_t
CO_CANmodule_init(
	CO_CANmodule_t* CANmodule,
	void* CANptr,
	CO_CANrx_t rxArray[], uint16_t rxSize,
	CO_CANtx_t txArray[], uint16_t txSize,
	uint16_t CANbitRate
	)
{
    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
        return CO_ERROR_ILLEGAL_ARGUMENT;

    /* Hold CANModule variable */
    CANmodule->CANptr = CANptr;

    /* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;

    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;

    CANmodule->useCANrxFilters = false; /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    /* Reset all variables */
    for (uint16_t i = 0U; i < rxSize; i++)
	{
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++)
        txArray[i].bufferFull = false;

    /***************************************/
    /* AT32 related configuration */
    /***************************************/
	if (CANbitRate != 0)
		((CANopenNodeAT32*) CANptr)->baudrate = CANbitRate;

	((CANopenNodeAT32*) CANptr)->HWInitFunction( CANbitRate );

    /*
     * Configure global filter that is used as last check if message did not pass any of other filters:
     *
     * We do not rely on hardware filters in this example
     * and are performing software filters instead
     *
     * Accept non-matching standard ID messages
     * Reject non-matching extended ID messages
     */

	// TODO: Not implemented

    return CO_ERROR_NO;
}

/******************************************************************************/
void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule)
{
    if (CANmodule != NULL &&
		CANmodule->CANptr != NULL &&
		((CANopenNodeAT32*)CANmodule->CANptr)->CANHandle != NULL
		)
	{
		can_reset( ((CANopenNodeAT32*)CANmodule->CANptr)->CANHandle );
	}
}

/******************************************************************************/
CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            __NOP();
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t*
CO_CANtxBufferInit(
	CO_CANmodule_t* CANmodule,
	uint16_t index,
	uint16_t ident,
	bool_t rtr,
	uint8_t noOfBytes,
	bool_t syncFlag
)
{
    CO_CANtx_t* buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize)
	{
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

#ifdef CAN_TX_IRQn
/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t
prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer)
{
    /* Check if TX FIFO is ready to accept more messages */
    static can_tx_message_type tx_hdr;

	/*
	 * RTR flag is part of identifier value
	 * hence it needs to be properly decoded
	 */
	tx_hdr.id_type = CAN_ID_STANDARD;
	tx_hdr.dlc = buffer->DLC;
	tx_hdr.standard_id = buffer->ident & CANID_MASK;
	tx_hdr.frame_type = (buffer->ident & FLAG_RTR) ? CAN_TFT_REMOTE : CAN_TFT_DATA;

	/* Now add message to FIFO. Should not fail */
	return can_message_transmit(
						((CANopenNodeAT32*)CANmodule->CANptr)->CANHandle,
						&tx_hdr
			) == CAN_TX_STATUS_NO_EMPTY ? 0 : 1;
}
#endif

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
	{
        if (!CANmodule->firstCANtxMessage)
		{
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

	static CAN_TX_QUEUE_TYPE item;

	item.id_type = CAN_ID_STANDARD;
	item.standard_id = buffer->ident & 0x7FF;
	item.dlc = buffer->DLC & 0xFF;
	item.frame_type = CAN_TFT_DATA;
	if (item.dlc != 0)
		memcpy((uint8_t*)&(item.data[0]), (uint8_t*)&buffer->data[0], item.dlc);
	if (CO_TxQueuePut(&item))
	{
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
	}
	else
	{
		buffer->bufferFull = true;
        CANmodule->CANtxCount++;
		err = CO_ERROR_TX_OVERFLOW;
	}
	return err;
#if 0
    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (prv_send_can_message(CANmodule, buffer))
	{
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
	else
	{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    return err;
#endif
}

/******************************************************************************/
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag)
	{
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0)
	{
        for (uint16_t i = CANmodule->txSize; i > 0U; --i)
            if (CANmodule->txArray[i].bufferFull &&
				CANmodule->txArray[i].syncFlag
			)
			{
				CANmodule->txArray[i].bufferFull = false;
				CANmodule->CANtxCount--;
				tpdoDeleted = 2U;
			}
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if ( tpdoDeleted != 0 )
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
}

/******************************************************************************/
#define CAN_ERR_ESTS_EAF	((uint32_t)0x00000001)
#define CAN_ERR_ESTS_EPF	((uint32_t)0x00000002)
#define CAN_ERR_ESTS_BOF	((uint32_t)0x00000004)
#define CAN_ERR_ESTS_ETR	((uint32_t)0x00000070)
#define CAN_ERR_ETR_NOERR			0x00	/*!< no error */
#define CAN_ERR_ETR_STUFFERR		0x01	/*!< stuff error */
#define CAN_ERR_ETR_FORMERR			0x02	/*!< form error */
#define CAN_ERR_ETR_ACKERR			0x03	/*!< acknowledgment error */
#define CAN_ERR_ETR_BITRECESSIVEERR	0x04	/*!< bit recessive error */
#define CAN_ERR_ETR_BITDOMINANTERR	0x05	/*!< bit dominant error */
#define CAN_ERR_ETR_CRCERR			0x06	/*!< crc error */
#define CAN_ERR_ETR_SOFTWARESETERR	0x07	/*!< software set error */

#define CAN_ERR_ESTS_TEC	((uint32_t)0xFF000000)
#define CAN_ERR_ESTS_REC	((uint32_t)0x00FF0000)
#define CAN_ERR_ESTS_ALL	(uint32_t)(CAN_ERR_ESTS_EAF | CAN_ERR_ESTS_EPF | CAN_ERR_ESTS_BOF | CAN_ERR_ESTS_ETR)

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule)
{
	uint16_t status;
	can_type* can = ((CANopenNodeAT32*)CANmodule->CANptr)->CANHandle;
	uint32_t err = can->ests & CAN_ERR_ESTS_ALL;
	if (err != 0)
		can_flag_clear(CAN_CAN, CAN_ETR_FLAG);

	if (CANmodule->errOld != err)
	{
		status = CANmodule->CANerrorStatus;
		CANmodule->errOld = err;
		if ( (err & CAN_ERR_ESTS_BOF) != 0 )
		{
			status |= CO_CAN_ERRTX_BUS_OFF;
			// In this driver, we assume that auto bus recovery is activated ! so this error will eventually handled automatically.
		}
		else
		{
			/* recalculate CANerrorStatus, first clear some flags */
			status &= ~(
						CO_CAN_ERRTX_BUS_OFF |
						CO_CAN_ERRRX_WARNING |
						CO_CAN_ERRRX_PASSIVE |
						CO_CAN_ERRTX_WARNING |
						CO_CAN_ERRTX_PASSIVE
			);

			if (err & CAN_ERR_ESTS_EAF)
				status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
			if (err & CAN_ERR_ESTS_EPF)
				status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
		}
		CANmodule->CANerrorStatus = status;
	}
}

#ifdef CAN_RX0_IRQn

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
CAN_RX0_IRQHandler ( void )
{
	static can_rx_message_type msg;

	if (can_interrupt_flag_get(CAN_CAN, CAN_RF0MN_FLAG) != RESET)
	{
		can_message_receive(CAN_CAN, CAN_RX_FIFO0, &msg);
		CO_RxQueuePut(&msg);
	}
}
#endif

#ifdef CAN_RX1_IRQn
/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
CAN_RX1_IRQHandler(can_type* hcan)
{
	static can_rx_message_type msg;

	if (can_interrupt_flag_get(CAN_CAN, CAN_RF1MN_FLAG) != RESET)
	{
		can_message_receive(CAN_CAN, CAN_RX_FIFO1, &msg);
		CO_RxQueuePut(&msg);
	}
}
#endif

#ifdef CAN_SE_IRQn
/**
  *  @brief  CAN interrupt function se
  *  @param  none
  *  @retval none
  */
void
CAN_SE_IRQHandler(void)
{
	if (can_interrupt_flag_get(CAN_CAN, CAN_ETR_FLAG) != RESET)
	{
		uint32_t err_index = CAN_CAN->ests & 0x70;
		can_flag_clear(CAN_CAN, CAN_ETR_FLAG);
		
		/* error type is stuff error */
		if (err_index == 0x10)
		{
			can_reset( CAN_CAN );
			can_configuration( ((CANopenNodeAT32*)(CANModule_local->CANptr))->baudrate );
		}
		else
		{
			can_transmit_cancel(CAN_CAN, CAN_TX_MAILBOX0 );
			can_transmit_cancel(CAN_CAN, CAN_TX_MAILBOX1 );
			can_transmit_cancel(CAN_CAN, CAN_TX_MAILBOX2 );
		}
	}
}
#endif

#ifdef CAN_TX_IRQn

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
void
CO_CANinterrupt_TX(CO_CANmodule_t* CANmodule, uint32_t MailboxNumber)
{
    CANmodule->firstCANtxMessage = false;            /* First CAN message (bootup) was sent successfully */
    CANmodule->bufferInhibitFlag = false;            /* Clear flag from previous message */
    if (CANmodule->CANtxCount > 0U)
	{	/* Are there any new messages waiting to be send */
        CO_CANtx_t* buffer = &CANmodule->txArray[0]; /* Start with first buffer handle */
        uint16_t i;

        /*
		 * Try to send more buffers, process all empty ones
		 *
		 * This function is always called from interrupt,
		 * however to make sure no preemption can happen, interrupts are anyway locked
		 * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
		 *  then no need to lock interrupts..)
		 */
        CO_LOCK_CAN_SEND(CANmodule);
        for (i = CANmodule->txSize; i > 0U; --i, ++buffer)
		{
            /* Try to send message */
            if (buffer->bufferFull)
			{
                if (prv_send_can_message(CANmodule, buffer))
				{
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U)
            CANmodule->CANtxCount = 0U;

        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

void
CAN_TX_IRQHandler( void )
{
	if ( can_flag_get(CAN_CAN, CAN_TM0TCF_FLAG) != RESET )
	{
		can_flag_clear(CAN_CAN, CAN_TM0TCF_FLAG);
		CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	}
	if ( can_flag_get(CAN_CAN, CAN_TM1TCF_FLAG) != RESET )
	{
		can_flag_clear(CAN_CAN, CAN_TM1TCF_FLAG);
		CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	}
	if ( can_flag_get(CAN_CAN, CAN_TM2TCF_FLAG) != RESET )
	{
		can_flag_clear(CAN_CAN, CAN_TM2TCF_FLAG);
		CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	}
}

#endif

void SendTxToUSB(CAN_TX_QUEUE_TYPE* msg);
void SendRxToUSB( CAN_RX_QUEUE_TYPE *msg );

void CO_process_queue()
{
	CAN_TX_QUEUE_TYPE* txQueueItem;
	can_trans_frame_type frame_type;

	while ((txQueueItem = CO_TxQueueGet()) != NULL)
	{
		frame_type = txQueueItem->frame_type;
		txQueueItem->frame_type = CAN_TFT_DATA;
		if (CAN_TX_STATUS_NO_EMPTY == can_message_transmit(CAN_CAN, txQueueItem))
			break;

		if (frame_type == CAN_TFT_DATA)
			SendTxToUSB(txQueueItem);

		CO_TxQueueShift();
	}

	CAN_RX_QUEUE_TYPE* rxQueueItem;
	CO_CANrx_t* msgBuff;
	uint16_t rxSize = CANModule_local->rxSize;
	uint16_t msg, index;

	while ((rxQueueItem = CANRxQueueGet()) != NULL)
	{
		frame_type = rxQueueItem->frame_type;
		rxQueueItem->frame_type = CAN_TFT_DATA;

		msg = rxQueueItem->standard_id;
		msgBuff = CANModule_local->rxArray;

		for (index = 0; index < rxSize; index++)
		{
			if (((msg ^ msgBuff->ident) & msgBuff->mask) == 0)
			{
				/* Call specific function, which will process the message */
				if (msgBuff->CANrx_callback)
					msgBuff->CANrx_callback(msgBuff->object, rxQueueItem);
				break;
			}
			msgBuff++;
		}

		if (frame_type == CAN_TFT_DATA)
			SendRxToUSB(rxQueueItem);

		CANRxQueueShift();
	}
}

