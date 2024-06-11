#include "301/CO_driver.h"
#include "CO_app_AT32.h"

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

void Timer_Init( )
{
	crm_clocks_freq_type crm_clocks_freq_struct = {0};

	/* get system clock */
	crm_clocks_freq_get(&crm_clocks_freq_struct);

	crm_periph_clock_enable(CAN_TIMER_CLOCK, TRUE);
	tmr_reset(CAN_TIMER);

	tmr_base_init(CAN_TIMER, 10 - 1, (crm_clocks_freq_struct.ahb_freq / 10000) - 1);
	tmr_cnt_dir_set(CAN_TIMER, TMR_COUNT_UP);
	tmr_clock_source_div_set( CAN_TIMER, TMR_CLOCK_DIV1 );
	tmr_flag_clear( CAN_TIMER, TMR_OVF_FLAG );

	/* overflow interrupt enable */
	tmr_interrupt_enable(CAN_TIMER, TMR_OVF_INT, TRUE);
	nvic_irq_enable(CAN_TIMER_IRQ, 1, 0);
	
	tmr_counter_enable(CAN_TIMER, TRUE);
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
    if (CANptr != NULL &&
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
    if (CANmodule->CANptr != NULL)
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
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate)
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

	((CANopenNodeAT32*) CANptr)->HWInitFunction( );

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
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

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

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

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
}

/******************************************************************************/
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0) {
        for (uint16_t i = CANmodule->txSize; i > 0U; --i) {
            if (CANmodule->txArray[i].bufferFull) {
                if (CANmodule->txArray[i].syncFlag) {
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
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
		can_flag_clear(CAN1, CAN_ETR_FLAG);

	if (CANmodule->errOld != err)
	{
		status = CANmodule->CANerrorStatus;
		CANmodule->errOld = err;
		if ((err & CAN_ERR_ESTS_BOF))
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

/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void prv_read_can_received_msg( can_rx_fifo_num_type fifo )
{
    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t* buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;            /* index of received message */
    uint32_t rcvMsgIdent;      /* identifier of the received message */
    
    static can_rx_message_type rx_hdr;

    /* Read received message from FIFO */
    can_message_receive( ((CANopenNodeAT32*)(CANModule_local->CANptr))->CANHandle, fifo, &rx_hdr);

    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.standard_id | (rx_hdr.frame_type == CAN_TFT_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = rx_hdr.dlc;
    rcvMsgIdent = rcvMsg.ident;

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANModule_local->useCANrxFilters)
	{
        __BKPT(0);
    }
	else
	{
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANModule_local->rxArray;
        for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer)
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
			{
				if ( buffer->CANrx_callback != NULL )
					buffer->CANrx_callback(buffer->object, (void*)&rcvMsg);
                break;
            }
	}
}

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
CAN1_RX0_IRQHandler ( void )
{
	if (can_interrupt_flag_get(CAN1, CAN_RF0MN_FLAG) != RESET)
		prv_read_can_received_msg( CAN_RX_FIFO0 );
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
CAN1_RX1_IRQHandler(can_type* hcan)
{
	if (can_interrupt_flag_get(CAN1, CAN_RF1MN_FLAG) != RESET)
		prv_read_can_received_msg( CAN_RX_FIFO1 );
}

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
    if (CANmodule->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */
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
        for (i = CANmodule->txSize; i > 0U; --i, ++buffer) {
            /* Try to send message */
            if (buffer->bufferFull) {
                if (prv_send_can_message(CANmodule, buffer)) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U) {
            CANmodule->CANtxCount = 0U;
        }
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

/**
  *  @brief  can1 interrupt function se
  *  @param  none
  *  @retval none
  */
void
CAN1_SE_IRQHandler(void)
{
	__IO uint32_t err_index = 0;
	if (can_interrupt_flag_get(CAN1, CAN_ETR_FLAG) != RESET)
	{
		err_index = CAN1->ests & 0x70;
		can_flag_clear(CAN1, CAN_ETR_FLAG);
		/* error type is stuff error */
		if (err_index == 0x00000010)
		{
			/* when stuff error occur: in order to ensure communication normally,
			user must restart can or send a frame of highest priority message here */
		}
	}
}

void
HAL_CAN_TxMailbox0CompleteCallback(can_type* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
}

void
HAL_CAN_TxMailbox1CompleteCallback(can_type* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
}

void
HAL_CAN_TxMailbox2CompleteCallback(can_type* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
}
