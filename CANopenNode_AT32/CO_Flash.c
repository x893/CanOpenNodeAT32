#include <string.h>

#include "CANopen.h"
#include "CO_app_AT32.h"
#include "storage/CO_storage.h"
#include "OD.h"
#include "CO_Flash.h"

#ifndef AT32F415RCT7
#error "Unknown CPU type"
#endif

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

#define OD_RUNTIME_START		((uint32_t)0x803E0000)
#define OD_FACTORY_START		((uint32_t)0x803F0000)
#define FLASH_PAGE_SIZE			0x800
#define PAGES_PER_STORAGE		2
#define CO_OD_FLASH_PARAM_SIZE	(FLASH_PAGE_SIZE * PAGES_PER_STORAGE)

#define OD_MAGIC_START			((uint32_t)0xCAFEBEEF)
#define OD_MAGIC_END			((uint32_t)0xBEEFCAFE)
#define MCU_ID1					((uint32_t)0x1FFFF7E8)

CO_storage_t storage;

const CO_storage_entry_t storageEntries[] = {
#define OD_RUNTIME_INDEX	0
	{.addr = &OD_PERSIST_COMM,
		.len = sizeof(OD_PERSIST_COMM),
		.subIndexOD = 2,
		.attr = CO_storage_cmd | CO_storage_restore,
		.addrNV = (void*)OD_RUNTIME_START
	},
#define OD_FACTORY_INDEX	1
	{.addr = &OD_PERSIST_COMM,
		.len = sizeof(OD_PERSIST_COMM),
		.subIndexOD = 4,
		.attr = CO_storage_restore,
		.addrNV = (void*)OD_FACTORY_START
	}
};

uint32_t storage_size(uint32_t size)
{
	return ((sizeof(Storage_Header_t) + size) + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);
}

static ODR_t store_storage(CO_storage_entry_t* entry, CO_CANmodule_t* CANmodule)
{
	(void)CANmodule;

	if (sizeof(OD_PERSIST_COMM_t) > CO_OD_FLASH_PARAM_SIZE)
	{
		ErrorHandler( );
	}

	if (storage_size(entry->len) * sizeof(uint32_t) > CO_OD_FLASH_PARAM_SIZE)
		return ODR_OUT_OF_MEM;

	Storage_Header_t header = {
		.OD_Length = entry->len,
		.NodeId = OD_PERSIST_COMM.x2101_nodeID,
		.Bitrate = OD_PERSIST_COMM.x2102_bitrate,
		.ProductCode = OD_PERSIST_COMM.x1018_identity.productCode,
		.RevisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
		.SerialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber,
		.OD_MagicStart = OD_MAGIC_START,
	};

	size_t len;
	uint32_t* src;
	uint32_t data;

	flash_status_type status = FLASH_PROGRAM_ERROR;
	flash_unlock();
	uint32_t flash_address = (uint32_t)entry->addrNV;

	do
	{
		status = flash_operation_wait_for(ERASE_TIMEOUT);
		if (status == FLASH_PROGRAM_ERROR || status == FLASH_EPP_ERROR)
			flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
		else if (status == FLASH_OPERATE_TIMEOUT)
			break;
#if PAGES_PER_STORAGE == 2
		status = flash_sector_erase(flash_address);
		if (status != FLASH_OPERATE_DONE)
			break;
		status = flash_sector_erase(flash_address + FLASH_PAGE_SIZE);
		if (status != FLASH_OPERATE_DONE)
			break;
#else
#error "Incorrect PAGES_PER_STORAGE"
#endif
		src = (uint32_t*)&header;
		len = sizeof(Storage_Header_t);
		while (len != 0)
		{
			status = flash_word_program(flash_address, *src++);
			if (status != FLASH_OPERATE_DONE)
				break;
			len -= sizeof(uint32_t);
			flash_address += sizeof(uint32_t);
		}
		if (status != FLASH_OPERATE_DONE)
			break;

		len = entry->len;
		src = entry->addr;
		while (len != 0)
		{
			if (len >= 4)
			{
				status = flash_word_program(flash_address, *src++);
				len -= sizeof(uint32_t);
			}
			else
			{
				data = 0;
				uint8_t* src_b = (uint8_t*)src;
				while (len != 0)
				{
					data = (data << 8) | *src_b++;
					--len;
				}
				status = flash_word_program(flash_address, data);
			}
			if (status != FLASH_OPERATE_DONE)
				break;
			flash_address += sizeof(uint32_t);
		}

		status = flash_word_program(flash_address, OD_MAGIC_END);
		if (status != FLASH_OPERATE_DONE)
			break;
		flash_address += sizeof(uint32_t);

	} while (0);
	flash_lock();

	return (status == FLASH_OPERATE_DONE)
		? ODR_OK
		: ODR_HW;
}

bool check_storage(CO_storage_entry_t* entry)
{
	register Storage_Flash_t* storage = (Storage_Flash_t*)entry->addrNV;
	do
	{
		if (storage->Header.OD_MagicStart != OD_MAGIC_START) break;
		if (storage->Header.OD_Length != entry->len) break;
		if (storage->Header.NodeId > 0x7F) break;
		if (storage->Header.Bitrate > 1000) break;
		if (storage->OD_MagicEnd != OD_MAGIC_END) break;
		return true;
	} while (0);
	return false;
}

static ODR_t restore_storage(CO_storage_entry_t* entry, CO_CANmodule_t* CANmodule)
{
	ODR_t result = ODR_GENERAL;
	if (check_storage(entry))
	{
		register Storage_Flash_t* storage = (Storage_Flash_t*)entry->addrNV;
		if (storage->Header.OD_Length == sizeof(OD_PERSIST_COMM_t))
		{
			memcpy(entry->addr, (void*)&storage->OD_PERSIST_COMM, sizeof(OD_PERSIST_COMM_t));
			result = ODR_OK;
		}
		OD_PERSIST_COMM.x2101_nodeID = storage->Header.NodeId;
		OD_PERSIST_COMM.x2102_bitrate = storage->Header.Bitrate;
		OD_PERSIST_COMM.x1018_identity.productCode = storage->Header.ProductCode;
		OD_PERSIST_COMM.x1018_identity.revisionNumber = storage->Header.RevisionNumber;
		OD_PERSIST_COMM.x1018_identity.serialNumber = storage->Header.SerialNumber;
	}
	return result;
}

static void Set_Serial(void)
{
	// Calculate serial number from chip unique id
	if (OD_PERSIST_COMM.x1018_identity.serialNumber == 0)
	{
		const uint32_t* Unique_ID = (const uint32_t*)MCU_ID1;
		crc_data_reset();
		OD_PERSIST_COMM.x1018_identity.serialNumber = crc_block_calculate((uint32_t*)Unique_ID, 3);
	}
}

CO_ReturnError_t CO_Storage_Init()
{
	CO_ReturnError_t err;

	if (ODR_OK != restore_storage((CO_storage_entry_t*)&storageEntries[OD_RUNTIME_INDEX], NULL))
	{
		restore_storage((CO_storage_entry_t*)&storageEntries[OD_FACTORY_INDEX], NULL);
	}
	Set_Serial();

	int storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);

	err = CO_storage_init(
		&storage,
		NULL,
		OD_ENTRY_H1010_storeParameters,
		OD_ENTRY_H1011_restoreDefaultParameters,
		store_storage,
		restore_storage,
		(CO_storage_entry_t*)storageEntries,
		storageEntriesCount
	);
	storage.enabled = (err == CO_ERROR_NO);
	return err;
}

void canopen_stotage_all()
{

}

void canopen_stotage_not_all()
{

}

#endif
