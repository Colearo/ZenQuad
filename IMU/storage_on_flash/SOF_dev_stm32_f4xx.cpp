/**
* @file SOF_dev_stm32.c
*
* @brief Flash device access interface for STM32 F4xx series
*
*
* History:
*/

#include <stdio.h>
#include "SOF_dev.h"
#include <string.h>

#include "mbed.h"
#include "stm32f4xx_hal_flash.h"

#define DCRLF	"\r\n"

#if 0
#define DPRINTF	printf
#define DASSERT(cond)  \
	if (!(cond)) { \
		printf("%s:%d assertion failed! '%s\r\n"\
			, __FILE__, __LINE__, #cond); \
	}
#else
#define DPRINTF(...)
#define DASSERT(...)
#endif


#if defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F407xx)
static const SOF_SectorSpec_t _sec_spec[] = {
    {FLASH_SECTOR_0, 0x08000000, 16*1024},
    {FLASH_SECTOR_1, 0x08004000, 16*1024},
    {FLASH_SECTOR_2, 0x08008000, 16*1024},
    {FLASH_SECTOR_3, 0x0800C000, 16*1024},
    {FLASH_SECTOR_4, 0x08010000, 64*1024},
    {FLASH_SECTOR_5, 0x08020000, 128*1024},
    {FLASH_SECTOR_6, 0x08040000, 128*1024},
    {FLASH_SECTOR_7, 0x08060000, 128*1024},
};
#else
#error "Not supported device"
#endif

#define N_SECTOR_SPEC			(sizeof(_sec_spec)/sizeof(_sec_spec[0]))

#define SECTOR_NO(sector)		_sec_spec[sector].sec_no
#define SECTOR_ADDR(sector)	_sec_spec[sector].sec_addr
#define SECTOR_SIZE(sector)		_sec_spec[sector].sec_size


static inline size_t handle_to_sector_index(SOF_DevHandle_t hdev)
{
    DASSERT(hdev < N_SECTOR_SPEC);
    return hdev;
}

const SOF_SectorSpec_t *SOF_dev_info(uint8_t sector_index)
{
    DASSERT(sector_index < N_SECTOR_SPEC);
    return &_sec_spec[sector_index];
}

int SOF_dev_is_valid_sector(uint8_t sector_index)
{
    return sector_index < N_SECTOR_SPEC;
}

const SOF_SectorSpec_t *SOF_dev_info_by_index(uint8_t sector_index)
{
    DASSERT(SOF_dev_is_valid_sector(sector_index));
    return &_sec_spec[sector_index];
}

const SOF_SectorSpec_t *SOF_dev_info(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);

    return SOF_dev_info_by_index(sector_index);
}

SOF_DevHandle_t SOF_dev_open(uint8_t sector_index)
{
    DASSERT(sector_index < N_SECTOR_SPEC);
    return (SOF_DevHandle_t)sector_index;
}

void SOF_dev_close(SOF_DevHandle_t hdev)
{
}

uint8_t *SOF_dev_get_hw_addr(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);

    return (uint8_t *)SECTOR_ADDR(sector_index);
}


void SOF_dev_erase(SOF_DevHandle_t hdev)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    FLASH_EraseInitTypeDef ei;
    uint32_t error = 0;

    DPRINTF("FLASH_Erase_Sector %d"DCRLF, SECTOR_NO(sector_index));
    HAL_FLASH_Unlock();

    ei.TypeErase = TYPEERASE_SECTORS;
    ei.Sector = SECTOR_NO(sector_index);
    ei.NbSectors = 1;
    ei.Banks = 0;
    ei.VoltageRange = VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&ei, &error);
    HAL_FLASH_Lock();
    DPRINTF("FLASH_Erase_Sector ok"DCRLF);
}


int SOF_dev_write_word(SOF_DevHandle_t hdev, uint32_t offset_addr, uint32_t data)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t dst = SECTOR_ADDR(sector_index) + offset_addr;

    DASSERT((offset_addr%sizeof(uint32_t)) == 0);
    HAL_FLASH_Unlock();
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, dst, data) != HAL_OK) {
        DPRINTF("FLASH_ProgramWord failed: %#x"DCRLF, dst);
        HAL_FLASH_Lock();
        return -1;
    }

    HAL_FLASH_Lock();

    if (data != SOF_dev_read_word(hdev, offset_addr)) {
        DPRINTF("addr=%x %#04x %#04x"DCRLF, dst, data, SOF_dev_read_word(hdev, offset_addr));
        return -1;
    }

    return 0;
}

uint32_t SOF_dev_read_word(SOF_DevHandle_t hdev, uint32_t offset_addr)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t src = SECTOR_ADDR(sector_index) + offset_addr;

    DASSERT((offset_addr%sizeof(uint32_t)) == 0);

    return *(volatile uint32_t*)src;
}

int SOF_dev_write_byte(SOF_DevHandle_t hdev, uint32_t offset_addr, uint8_t data)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t dst = SECTOR_ADDR(sector_index) + offset_addr;

    HAL_FLASH_Unlock();
    if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, dst, data) != HAL_OK) {
        DPRINTF("FLASH_ProgramWord failed: %#x"DCRLF, dst);
        HAL_FLASH_Lock();
        return -1;
    }

    HAL_FLASH_Lock();

    if (data != SOF_dev_read_byte(hdev, offset_addr)) {
        DPRINTF("addr=%x %#02x %#02x"DCRLF, dst, data, SOF_dev_read_byte(hdev, offset_addr));
        return -1;
    }

    return 0;
}

uint8_t SOF_dev_read_byte(SOF_DevHandle_t hdev, uint32_t offset_addr)
{
    uint8_t sector_index = handle_to_sector_index(hdev);
    uint32_t src = SECTOR_ADDR(sector_index) + offset_addr;

    return *(volatile uint8_t*)src;
}


