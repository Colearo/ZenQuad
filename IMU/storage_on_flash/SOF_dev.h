/**
* @file SOF_dev.c
*
* @author hillkim7@gmail.com
* @brief Storage On Flash device interface
*
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

/** The default value of erased flash memory */
#define SOF_ERASED_BYTE_VALUE	0xFF

#define SOF_INVALID_HANDLE	0xFFFFFFFF

/** Error code definition */
typedef enum {
    kSOF_ErrNone = 0,
    kSOF_ErrParam,
    kSOF_ErrNoInfo,
    kSOF_ErrBusyBlock,
    kSOF_ErrBadBlock,
    kSOF_ErrDataCurrupted,
    kSOF_ErrFatal,
} SOF_Error_t;;

/** Flash sector specification */
typedef struct {
    uint32_t	sec_no;
    uint32_t	sec_addr;
    uint32_t	sec_size;
} SOF_SectorSpec_t;

/** statics of SOF block */
typedef struct {
    uint8_t		*data_addr;
    uint32_t	data_size;
    uint32_t	free_size;
} SOF_Statics_t;

typedef uint32_t SOF_DevHandle_t;

class SOF_BlockHandle;
typedef SOF_BlockHandle* SOF_BlockHandle_t;

/*-----------------------------------------------------------------------------------------------------*/
/* Flash device interface */
/*-----------------------------------------------------------------------------------------------------*/
int SOF_dev_is_valid_sector(uint8_t sector_index);
const SOF_SectorSpec_t *SOF_dev_info(SOF_DevHandle_t hdev);
const SOF_SectorSpec_t *SOF_dev_info_by_index(uint8_t sector_index);
SOF_DevHandle_t SOF_dev_open(uint8_t sector_index);
void SOF_dev_close(SOF_DevHandle_t hdev);
uint8_t *SOF_dev_get_hw_addr(SOF_DevHandle_t hdev);
void SOF_dev_erase(SOF_DevHandle_t handle);
int SOF_dev_write_word(SOF_DevHandle_t handle, uint32_t offset_addr, uint32_t data);
uint32_t SOF_dev_read_word(SOF_DevHandle_t handle, uint32_t offset_addr);
int SOF_dev_write_byte(SOF_DevHandle_t handle, uint32_t offset_addr, uint8_t data);
uint8_t SOF_dev_read_byte(SOF_DevHandle_t handle, uint32_t offset_addr);

/*-----------------------------------------------------------------------------------------------------*/
/* Flash block device interface */
/*-----------------------------------------------------------------------------------------------------*/
bool SOF_block_format(uint8_t sector_index);
SOF_BlockHandle_t SOF_block_open_storage(uint8_t sector_index, SOF_Error_t *err);
SOF_BlockHandle_t SOF_block_create_storage(uint8_t sector_index, SOF_Error_t *err);
bool SOF_block_close(SOF_BlockHandle_t handle);
uint8_t *SOF_block_base_addr(SOF_BlockHandle_t handle);
bool SOF_block_putc(SOF_BlockHandle_t handle, uint8_t c);
size_t SOF_block_write(SOF_BlockHandle_t handle, const uint8_t *p, size_t p_size);
bool SOF_block_getc(SOF_BlockHandle_t handle, uint8_t *c);
size_t SOF_block_read(SOF_BlockHandle_t handle, uint8_t *p, size_t p_size);
size_t SOF_block_get_free_size(SOF_BlockHandle_t handle);
uint32_t SOF_block_storage_size(SOF_BlockHandle_t handle);
SOF_Error_t SOF_block_get_statics(uint8_t sector_index, SOF_Statics_t *stat);
const SOF_SectorSpec_t *SOF_block_get_info(uint8_t sector_index);


