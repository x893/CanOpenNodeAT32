#ifndef CO_FLASH_H
#define CO_FLASH_H

#include "storage/CO_storage.h"
#include "301/CO_ODinterface.h"
#include "OD.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) || defined CO_DOXYGEN

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct {
		uint32_t OD_Length;			// OD_PERSIST_ROM size in bytes
		uint16_t NodeId;
		uint16_t Bitrate;
		uint32_t ProductCode;
		uint32_t RevisionNumber;
		uint32_t SerialNumber;
		uint32_t OD_MagicStart;
	} Storage_Header_t;

	typedef struct {
		Storage_Header_t Header;
		OD_PERSIST_COMM_t OD_PERSIST_COMM;
		uint32_t OD_MagicEnd;
	} Storage_Flash_t;

	CO_ReturnError_t CO_Storage_Init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
#endif /* CO_STORAGE_BLANK_H */
