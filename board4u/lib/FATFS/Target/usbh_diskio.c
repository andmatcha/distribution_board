/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbh_diskio.c (based on usbh_diskio_template.c v2.0.2)
  * @brief   USB Host Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "ff_gen_drv.h"
#include "usbh_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define USB_DEFAULT_BLOCK_SIZE 512
#define USBH_DISK_MAX_LUNS 4U
#define MBR_SIGNATURE_OFFSET 510U
#define MBR_PARTITION_TABLE_OFFSET 446U
#define MBR_PARTITION_ENTRY_SIZE 16U
#define MBR_PARTITION_TYPE_OFFSET 4U
#define GPT_PROTECTIVE_PARTITION_TYPE 0xEEU
#define GPT_HEADER_LBA 1U
#define GPT_HEADER_SIGNATURE "EFI PART"
#define GPT_HEADER_PARTITION_ENTRIES_LBA_OFFSET 72U
#define GPT_HEADER_PARTITION_COUNT_OFFSET 80U
#define GPT_HEADER_PARTITION_ENTRY_SIZE_OFFSET 84U
#define GPT_ENTRY_TYPE_GUID_OFFSET 0U
#define GPT_ENTRY_FIRST_LBA_OFFSET 32U
#define GPT_ENTRY_LAST_LBA_OFFSET 40U
#define GPT_GUID_SIZE 16U
#define FAT_JUMP_BOOT_E9 0xE9U
#define FAT_JUMP_BOOT_EB 0xEBU
#define FAT_JUMP_BOOT_NOP 0x90U
#define FAT_FILESYSTEM_TYPE_OFFSET 54U
#define FAT32_FILESYSTEM_TYPE_OFFSET 82U

typedef struct
{
  uint8_t initialized;
  uint16_t sector_size;
  uint32_t physical_sector_count;
  uint32_t logical_sector_count;
  uint32_t partition_offset;
} USBH_DiskContext;

/* Private variables ---------------------------------------------------------*/
extern USBH_HandleTypeDef  hUSB_Host;
static USBH_DiskContext usbh_disk_context[USBH_DISK_MAX_LUNS];
static uint8_t usbh_sector_buffer[_MAX_SS];
static uint8_t usbh_partition_buffer[_MAX_SS];

/* Private function prototypes -----------------------------------------------*/
DSTATUS USBH_initialize (BYTE);
DSTATUS USBH_status (BYTE);
DRESULT USBH_read (BYTE, BYTE*, DWORD, UINT);

#if _USE_WRITE == 1
  DRESULT USBH_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
  DRESULT USBH_ioctl (BYTE, BYTE, void*);
#endif /* _USE_IOCTL == 1 */

static uint16_t usbh_load_le16(const uint8_t *data);
static uint32_t usbh_load_le32(const uint8_t *data);
static uint64_t usbh_load_le64(const uint8_t *data);
static uint8_t usbh_guid_is_zero(const uint8_t *guid);
static uint8_t usbh_sector_looks_like_fat(const uint8_t *sector);
static uint8_t usbh_has_protective_mbr(const uint8_t *sector);
static DRESULT usbh_read_raw_sector(BYTE lun, DWORD sector, uint8_t *buffer);
static DSTATUS usbh_initialize_context(BYTE lun);
static void usbh_reset_context(BYTE lun);
static USBH_DiskContext *usbh_get_context(BYTE lun);
static uint8_t usbh_try_map_gpt_partition(BYTE lun, USBH_DiskContext *context);

const Diskio_drvTypeDef  USBH_Driver =
{
  USBH_initialize,
  USBH_status,
  USBH_read,
#if  _USE_WRITE == 1
  USBH_write,
#endif /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USBH_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static uint16_t usbh_load_le16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t usbh_load_le32(const uint8_t *data)
{
  return (uint32_t)data[0]
       | ((uint32_t)data[1] << 8U)
       | ((uint32_t)data[2] << 16U)
       | ((uint32_t)data[3] << 24U);
}

static uint64_t usbh_load_le64(const uint8_t *data)
{
  return (uint64_t)usbh_load_le32(data)
       | ((uint64_t)usbh_load_le32(data + 4) << 32U);
}

static uint8_t usbh_guid_is_zero(const uint8_t *guid)
{
  uint32_t index;

  for (index = 0U; index < GPT_GUID_SIZE; index++)
  {
    if (guid[index] != 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t usbh_sector_looks_like_fat(const uint8_t *sector)
{
  if (usbh_load_le16(sector + MBR_SIGNATURE_OFFSET) != 0xAA55U)
  {
    return 0U;
  }

  if ((sector[0] == FAT_JUMP_BOOT_E9)
   || ((sector[0] == FAT_JUMP_BOOT_EB) && (sector[2] == FAT_JUMP_BOOT_NOP)))
  {
    if ((memcmp(sector + FAT_FILESYSTEM_TYPE_OFFSET, "FAT", 3U) == 0)
     || (memcmp(sector + FAT32_FILESYSTEM_TYPE_OFFSET, "FAT3", 4U) == 0))
    {
      return 1U;
    }
  }

  if (memcmp(sector, "\xEB\x76\x90" "EXFAT   ", 11U) == 0)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t usbh_has_protective_mbr(const uint8_t *sector)
{
  uint32_t entry_index;

  if (usbh_load_le16(sector + MBR_SIGNATURE_OFFSET) != 0xAA55U)
  {
    return 0U;
  }

  for (entry_index = 0U; entry_index < 4U; entry_index++)
  {
    const uint8_t *entry = sector + MBR_PARTITION_TABLE_OFFSET + (entry_index * MBR_PARTITION_ENTRY_SIZE);

    if (entry[MBR_PARTITION_TYPE_OFFSET] == GPT_PROTECTIVE_PARTITION_TYPE)
    {
      return 1U;
    }
  }

  return 0U;
}

static DRESULT usbh_read_raw_sector(BYTE lun, DWORD sector, uint8_t *buffer)
{
  return (USBH_MSC_Read(&hUSB_Host, lun, sector, buffer, 1U) == USBH_OK) ? RES_OK : RES_ERROR;
}

static void usbh_reset_context(BYTE lun)
{
  if (lun < USBH_DISK_MAX_LUNS)
  {
    (void)memset(&usbh_disk_context[lun], 0, sizeof(usbh_disk_context[lun]));
  }
}

static USBH_DiskContext *usbh_get_context(BYTE lun)
{
  if (lun >= USBH_DISK_MAX_LUNS)
  {
    return NULL;
  }

  return &usbh_disk_context[lun];
}

static uint8_t usbh_try_map_gpt_partition(BYTE lun, USBH_DiskContext *context)
{
  uint64_t partition_entries_lba;
  uint32_t partition_count;
  uint32_t partition_entry_size;
  uint32_t entries_per_sector;
  uint32_t partition_index;
  uint32_t sector_index = UINT32_MAX;
  uint64_t best_first_lba = 0U;
  uint64_t best_sector_count = 0U;
  uint32_t candidate_count = 0U;

  if (usbh_read_raw_sector(lun, 0U, usbh_sector_buffer) != RES_OK)
  {
    return 0U;
  }

  if (usbh_has_protective_mbr(usbh_sector_buffer) == 0U)
  {
    return 0U;
  }

  if (usbh_read_raw_sector(lun, GPT_HEADER_LBA, usbh_sector_buffer) != RES_OK)
  {
    return 0U;
  }

  if (memcmp(usbh_sector_buffer, GPT_HEADER_SIGNATURE, 8U) != 0)
  {
    return 0U;
  }

  partition_entries_lba = usbh_load_le64(usbh_sector_buffer + GPT_HEADER_PARTITION_ENTRIES_LBA_OFFSET);
  partition_count = usbh_load_le32(usbh_sector_buffer + GPT_HEADER_PARTITION_COUNT_OFFSET);
  partition_entry_size = usbh_load_le32(usbh_sector_buffer + GPT_HEADER_PARTITION_ENTRY_SIZE_OFFSET);

  if ((partition_entries_lba > UINT32_MAX)
   || (partition_count == 0U)
   || (partition_entry_size < 56U)
   || (partition_entry_size > context->sector_size))
  {
    return 0U;
  }

  entries_per_sector = context->sector_size / partition_entry_size;
  if (entries_per_sector == 0U)
  {
    return 0U;
  }

  for (partition_index = 0U; partition_index < partition_count; partition_index++)
  {
    const uint8_t *entry;
    uint32_t next_sector_index = partition_index / entries_per_sector;
    uint64_t first_lba;
    uint64_t last_lba;
    uint64_t sector_count;

    if (next_sector_index != sector_index)
    {
      if (usbh_read_raw_sector(lun,
                               (DWORD)partition_entries_lba + next_sector_index,
                               usbh_sector_buffer) != RES_OK)
      {
        return 0U;
      }
      sector_index = next_sector_index;
    }

    entry = usbh_sector_buffer + ((partition_index % entries_per_sector) * partition_entry_size);
    if (usbh_guid_is_zero(entry + GPT_ENTRY_TYPE_GUID_OFFSET) != 0U)
    {
      continue;
    }

    first_lba = usbh_load_le64(entry + GPT_ENTRY_FIRST_LBA_OFFSET);
    last_lba = usbh_load_le64(entry + GPT_ENTRY_LAST_LBA_OFFSET);
    if ((first_lba == 0U)
     || (first_lba > last_lba)
     || (last_lba >= context->physical_sector_count))
    {
      continue;
    }

    sector_count = (last_lba - first_lba) + 1U;
    if ((first_lba > UINT32_MAX) || (sector_count > UINT32_MAX))
    {
      continue;
    }

    if (usbh_read_raw_sector(lun, (DWORD)first_lba, usbh_partition_buffer) != RES_OK)
    {
      continue;
    }

    if (usbh_sector_looks_like_fat(usbh_partition_buffer) == 0U)
    {
      continue;
    }

    candidate_count++;
    if (sector_count > best_sector_count)
    {
      best_first_lba = first_lba;
      best_sector_count = sector_count;
    }
  }

  if (best_sector_count != 0U)
  {
    context->partition_offset = (uint32_t)best_first_lba;
    context->logical_sector_count = (uint32_t)best_sector_count;
    printf("[usb-storage] GPT partition mapped: start=%lu sectors, size=%lu sectors, candidates=%lu\r\n",
           (unsigned long)context->partition_offset,
           (unsigned long)context->logical_sector_count,
           (unsigned long)candidate_count);
    return 1U;
  }

  printf("[usb-storage] GPT detected but no FAT/exFAT partition was found\r\n");
  return 0U;
}

static DSTATUS usbh_initialize_context(BYTE lun)
{
  MSC_LUNTypeDef info;
  USBH_DiskContext *context = usbh_get_context(lun);

  if ((context == NULL) || (USBH_MSC_UnitIsReady(&hUSB_Host, lun) == 0U))
  {
    return STA_NOINIT;
  }

  if (USBH_MSC_GetLUNInfo(&hUSB_Host, lun, &info) != USBH_OK)
  {
    return STA_NOINIT;
  }

  if ((info.capacity.block_size < _MIN_SS)
   || (info.capacity.block_size > _MAX_SS)
   || ((info.capacity.block_size & (info.capacity.block_size - 1U)) != 0U))
  {
    printf("[usb-storage] unsupported sector size: %u\r\n", (unsigned int)info.capacity.block_size);
    return STA_NOINIT;
  }

  context->initialized = 1U;
  context->sector_size = info.capacity.block_size;
  context->physical_sector_count = info.capacity.block_nbr + 1U;
  context->logical_sector_count = context->physical_sector_count;
  context->partition_offset = 0U;

  (void)usbh_try_map_gpt_partition(lun, context);

  return 0U;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : lun id
  * @retval DSTATUS: Operation status
  */
DSTATUS USBH_initialize(BYTE lun)
{
  /* CAUTION : USB Host library has to be initialized in the application */

  return usbh_initialize_context(lun);
}

/**
  * @brief  Gets Disk Status
  * @param  lun : lun id
  * @retval DSTATUS: Operation status
  */
DSTATUS USBH_status(BYTE lun)
{
  if (USBH_MSC_UnitIsReady(&hUSB_Host, lun) == 0U)
  {
    usbh_reset_context(lun);
    return STA_NOINIT;
  }

  return 0U;
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */

/**
  * @brief  Reads Sector(s)
  * @param  lun : lun id
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USBH_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  MSC_LUNTypeDef info;
  USBH_DiskContext *context = usbh_get_context(lun);
  DWORD physical_sector = sector;

  if ((context == NULL) || (context->initialized == 0U))
  {
    if (usbh_initialize_context(lun) != 0U)
    {
      return RES_NOTRDY;
    }
    context = usbh_get_context(lun);
  }

  if ((context == NULL) || (context->initialized == 0U))
  {
    return RES_NOTRDY;
  }

  if (((uint64_t)sector + count) > context->logical_sector_count)
  {
    return RES_PARERR;
  }

  physical_sector += context->partition_offset;

  if(USBH_MSC_Read(&hUSB_Host, lun, physical_sector, buff, count) == USBH_OK)
  {
    res = RES_OK;
  }
  else
  {
    USBH_MSC_GetLUNInfo(&hUSB_Host, lun, &info);

    switch (info.sense.asc)
    {
    case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
    case SCSI_ASC_MEDIUM_NOT_PRESENT:
    case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
      USBH_ErrLog ("USB Disk is not ready!");
      res = RES_NOTRDY;
      break;

    default:
      res = RES_ERROR;
      break;
    }
  }

  return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */

/**
  * @brief  Writes Sector(s)
  * @param  lun : lun id
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USBH_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  MSC_LUNTypeDef info;
  USBH_DiskContext *context = usbh_get_context(lun);
  DWORD physical_sector = sector;

  if ((context == NULL) || (context->initialized == 0U))
  {
    if (usbh_initialize_context(lun) != 0U)
    {
      return RES_NOTRDY;
    }
    context = usbh_get_context(lun);
  }

  if ((context == NULL) || (context->initialized == 0U))
  {
    return RES_NOTRDY;
  }

  if (((uint64_t)sector + count) > context->logical_sector_count)
  {
    return RES_PARERR;
  }

  physical_sector += context->partition_offset;

  if(USBH_MSC_Write(&hUSB_Host, lun, physical_sector, (BYTE *)buff, count) == USBH_OK)
  {
    res = RES_OK;
  }
  else
  {
    USBH_MSC_GetLUNInfo(&hUSB_Host, lun, &info);

    switch (info.sense.asc)
    {
    case SCSI_ASC_WRITE_PROTECTED:
      USBH_ErrLog("USB Disk is Write protected!");
      res = RES_WRPRT;
      break;

    case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
    case SCSI_ASC_MEDIUM_NOT_PRESENT:
    case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
      USBH_ErrLog("USB Disk is not ready!");
      res = RES_NOTRDY;
      break;

    default:
      res = RES_ERROR;
      break;
    }
  }

  return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */

/**
  * @brief  I/O control operation
  * @param  lun : lun id
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USBH_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  MSC_LUNTypeDef info;
  USBH_DiskContext *context = usbh_get_context(lun);

  if ((context == NULL) || (context->initialized == 0U))
  {
    if (usbh_initialize_context(lun) != 0U)
    {
      return RES_NOTRDY;
    }
    context = usbh_get_context(lun);
  }

  if ((context == NULL) || (context->initialized == 0U))
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC:
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    *(DWORD*)buff = context->logical_sector_count;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    *(WORD*)buff = context->sector_size;
    res = RES_OK;
    break;

    /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    (void)info;
    *(DWORD*)buff = 1U;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */
