#include <stdio.h>

#include "CO_app_AT32.h"
#include "CANopen.h"
#include "CO_storageBlank.h"
#include "OD.h"

// It will be set by canopen_app_init and will be used across app to get access to CANOpen objects
CANopenNodeAT32* pCANopenNodeAT32;

/* default values for CO_CANopenInit() */
#define NMT_CONTROL		(CO_NMT_control_t)( \
    CO_NMT_STARTUP_TO_OPERATIONAL |	\
    CO_NMT_ERR_ON_ERR_REG |			\
	CO_ERR_REG_GENERIC_ERR |		\
	CO_ERR_REG_COMMUNICATION		\
	)
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

/* Global variables and objects */
CO_t* CO = NULL; /* CANopen object */

// Global variables
CO_ReturnError_t err;

static bool_t LSScfgStoreCallback(void *object, uint8_t id, uint16_t bitRate)
{
	((CANopenNodeAT32 *)object)->baudrate = bitRate;
	((CANopenNodeAT32 *)object)->desiredNodeID = id;
	return true;
}

int can_get_bitrate_index(uint16_t bitRate);
bool_t CO_LSSchkBitrateCallback(void *object, uint16_t bitRate)
{
    (void)object;
	return (can_get_bitrate_index(bitRate) >= 0);
}

/* This function will basically setup the CANopen node */
int
canopen_app_init(CANopenNodeAT32* _canopenNodeAT32)
{
	// Keep a copy global reference of canOpenAT32 Object
	pCANopenNodeAT32 = _canopenNodeAT32;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                            .len = sizeof(OD_PERSIST_COMM),
                                            .subIndexOD = 2,
                                            .attr = CO_storage_cmd | CO_storage_restore,
                                            .addrNV = NULL}};
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Allocate memory */
    CO_config_t* config_ptr = NULL;

#ifdef CO_MULTIPLE_OD
	#error "Never check CO_MULTIPLE_OD"
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL)
	{
        ErrorHandler( );
    }
	
    pCANopenNodeAT32->canOpenStack = CO;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule,
			OD_ENTRY_H1010_storeParameters,
			OD_ENTRY_H1011_restoreDefaultParameters,
			storageEntries, storageEntriesCount,
			&storageInitError
	);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT)
	{
        log_printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif

    /* CANopen communication reset - initialize CANopen objects *******************/
    CO->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode( (void*)pCANopenNodeAT32 );
    CO_CANmodule_disable( CO->CANmodule );

    /* initialize CANopen */
    err = CO_CANinit( CO, (void*)pCANopenNodeAT32, pCANopenNodeAT32->baudrate );
    if (err != CO_ERROR_NO)
	{
		ErrorHandler( );
    }

    CO_LSS_address_t lssAddress = {
		.identity = {
			.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
			.productCode = OD_PERSIST_COMM.x1018_identity.productCode,
			.revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
			.serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
		}
	};
    err = CO_LSSinit(CO, &lssAddress, &pCANopenNodeAT32->desiredNodeID, &pCANopenNodeAT32->baudrate);
    if (err != CO_ERROR_NO)
	{
		ErrorHandler( );
    }

    pCANopenNodeAT32->activeNodeID = pCANopenNodeAT32->desiredNodeID;
    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         pCANopenNodeAT32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
	{
		ErrorHandler( );
    }

	/* initialize callbacks */
	CO_LSSslave_initCheckBitRateCallback( CO->LSSslave, NULL, CO_LSSchkBitrateCallback );
	CO_LSSslave_initCfgStoreCallback( CO->LSSslave, pCANopenNodeAT32, LSScfgStoreCallback );

    err = CO_CANopenInitPDO(CO, CO->em, OD, pCANopenNodeAT32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO)
	{
		ErrorHandler( );
    }

    /* Configure Timer interrupt function for execution every 1 millisecond */
	tmr_interrupt_enable( pCANopenNodeAT32->timerHandle, TMR_OVF_INT, TRUE );

    /* Configure CAN transmit and receive interrupt */

    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured)
	{
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0)
		{
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    }

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule);

    return 0;
}

void canopen_app_process( uint32_t timeDifference_us )
{
	CO_NMT_reset_cmd_t reset_status;
	bool_t syncWas;

    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */

	CO_process_queue( );

	reset_status = CO_process(CO, false, timeDifference_us, NULL);

	if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal)
	{
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
		syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#else
		syncWas = false;
#endif

#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
		CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif

		#warning "Insert logic here"

#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
		CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif
	/* Further I/O or nonblocking application code may go here. */
	}

	CO_process_queue( );

	if (reset_status == CO_RESET_COMM)
	{
		/* delete objects from memory */
		tmr_counter_enable( pCANopenNodeAT32->timerHandle, FALSE);
		CO_CANsetConfigurationMode((void*)pCANopenNodeAT32);
		CO_delete(CO);
		canopen_app_init(pCANopenNodeAT32); // Reset Communication routine
	}
	else if (reset_status == CO_RESET_APP)
	{
		NVIC_SystemReset( );
	}
}
