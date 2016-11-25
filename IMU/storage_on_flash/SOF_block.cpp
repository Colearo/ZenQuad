/*
* @file SOF_block.cpp
*
* @brief MTD device handling of STM32 internal flash memory.
*
*
* History:
*/

#include <stdio.h>
#include <assert.h>
#include "SOF_dev.h"
#include <string.h>

#define LOCAL_DEBUG 0 // turn on local debug

#define DCRLF	"\r\n"

#if LOCAL_DEBUG
#define DPRINTF printf
#define EPRINTF printf
#define DUMP_BLOCK dump_block
#define DASSERT assert
#else
#define DPRINTF(...)
#define EPRINTF(...)
#define DUMP_BLOCK(...)
#define DASSERT(...)
#endif

static uint16_t checksum(const uint8_t* data, int count);

#define SYNC_MARK_BYTE_IN_LEN	0x07	// signature mark for upper byte in the storage_len

#define RESERVED_BLOCK_INFO_SIZE sizeof(BlockInfo_t)

typedef struct {
    uint16_t for_storage;
    uint16_t for_info;
} BlockChecksum_t;

typedef struct {
    BlockChecksum_t		csum;
    uint32_t			storage_len;
} BlockInfo_t;

typedef struct {
    uint32_t	begin_offset;
    uint32_t	len;
    uint16_t	storage_csum;
} StorageInfo_t;

class SOF_BlockHandle
{
public:
    SOF_BlockHandle()
        : write_mode_(false)
        , cur_pos_(0)
        , hdev_(SOF_INVALID_HANDLE)
        , storage_max_offset_(0)
        , storage_begin_offset_(0)
        , storage_end_offset_(0) {
    }

    bool is_writable() const {
        return write_mode_;
    }

    uint32_t total_physical_block_size() const {
        return SOF_dev_info(hdev_)->sec_size;
    }

public:
    bool				write_mode_;
    size_t				cur_pos_;
    SOF_DevHandle_t		hdev_;
    uint32_t			storage_max_offset_;
    uint32_t			storage_begin_offset_;
    uint32_t			storage_end_offset_;
};


static bool get_block_info(const SOF_BlockHandle_t handle, size_t seq, BlockInfo_t *info, uint32_t *loc_offset)
{
    uint32_t check_pos = ((seq+1) * sizeof(BlockInfo_t));
    uint32_t info_pos;

    DASSERT(check_pos < handle->total_physical_block_size());
    if (check_pos >= handle->total_physical_block_size())
        return false;

    *loc_offset = info_pos = handle->total_physical_block_size() - check_pos;

    // checksum in the first word
    *((uint32_t*)&info->csum) = SOF_dev_read_word(handle->hdev_, info_pos);
    // storage len in the next word
    info->storage_len = SOF_dev_read_word(handle->hdev_, info_pos + 4);

    return true;
}

static bool is_empty_block_info(BlockInfo_t *info)
{
    uint8_t *p = (uint8_t*)info;

    for (size_t i = 0; i < sizeof(BlockInfo_t); ++i)
        if (p[i] != SOF_ERASED_BYTE_VALUE)
            return false;

    return true;
}

static bool is_valid_block_info(BlockInfo_t *info)
{
    uint16_t csum = checksum((uint8_t*)&info->storage_len, 4);

    if (SYNC_MARK_BYTE_IN_LEN != (info->storage_len >> 24)) {
        EPRINTF("no sync mark in storage_len=%#x"DCRLF,info->storage_len);
        return false;
    }

    if (csum != info->csum.for_info) {
        EPRINTF("CSUM mismatch %#x %#x"DCRLF,csum, info->csum.for_info);
        return false;
    }

    return true;
}

static bool get_empty_info_location(const SOF_BlockHandle_t handle, uint32_t *loc_offset)
{
    BlockInfo_t info;
    uint32_t pos;

    for (size_t seq = 0; get_block_info(handle, seq, &info, &pos); ++seq) {
        //DPRINTF("[%u] len=%#x pos=%u"DCRLF,seq, info.storage_len, pos);
        if (is_empty_block_info(&info)) {
            *loc_offset = pos;
            return true;
        }
    }

    return false;
}

static SOF_Error_t probe_active_storage_info(const SOF_BlockHandle_t handle, StorageInfo_t *storage_info)
{
    BlockInfo_t info, last_info;
    uint32_t pos;
    uint32_t storage_len_sum = 0;

    for (size_t seq = 0; get_block_info(handle, seq, &info, &pos); ++seq) {
        if (is_empty_block_info(&info)) {
            if (seq == 0)
                return kSOF_ErrNoInfo;
            break;
        }
        if (!is_valid_block_info(&info)) {
            if (storage_info->begin_offset + storage_info->len == pos) {
                DPRINTF("data is full: %u"DCRLF,storage_info->begin_offset + storage_info->len);
                break;
            }

            EPRINTF("invalid block at %u"DCRLF,pos);
            return kSOF_ErrBadBlock;
        }

        storage_len_sum += info.storage_len & 0x00FFFFFF;
        last_info = info;
    }

    uint32_t storage_len = last_info.storage_len & 0x00FFFFFF;

    storage_info->begin_offset = storage_len_sum - storage_len;
    storage_info->len = storage_len;
    storage_info->storage_csum = last_info.csum.for_storage;

    return kSOF_ErrNone;
}


#if LOCAL_DEBUG
static void dump_block(SOF_BlockHandle_t handle)
{
    DPRINTF("sector(%u)"DCRLF, SOF_dev_info(handle->hdev_)->sec_no);
    DPRINTF("	offset               =%u"DCRLF, handle->cur_pos_);
    DPRINTF("	writemode            =%d"DCRLF, handle->write_mode_);
    DPRINTF("	storage_max_offset   =%u"DCRLF, handle->storage_max_offset_);
    DPRINTF("	storage_begin_offset =%u"DCRLF, handle->storage_begin_offset_);
    DPRINTF("	storage_end_offset   =%u"DCRLF, handle->storage_end_offset_);
    DPRINTF("	free=%u total=%u"DCRLF,SOF_block_get_free_size(handle), handle->total_physical_block_size());
}
#endif

size_t SOF_block_get_free_size(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    if (handle->storage_end_offset_ <= handle->storage_max_offset_-RESERVED_BLOCK_INFO_SIZE)
        return (handle->storage_max_offset_- RESERVED_BLOCK_INFO_SIZE) - handle->storage_end_offset_;
    else {
        return 0;
    }
}

uint32_t SOF_block_storage_size(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    return handle->storage_end_offset_ - handle->storage_begin_offset_;
}

static uint16_t checksum(const uint8_t* data, int count)
{
    // Fletcher's checksum algorithm
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;

    for( index = 0; index < count; ++index ) {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    return (sum2 << 8) | sum1;
}

static uint16_t compute_storage_checksum(SOF_BlockHandle_t handle)
{
    uint8_t *addr = SOF_dev_get_hw_addr(handle->hdev_);

    return checksum(addr+handle->storage_begin_offset_, SOF_block_storage_size(handle));
}

static bool write_storage_info(SOF_BlockHandle_t handle)
{
    BlockInfo_t cs;

    cs.storage_len = (SYNC_MARK_BYTE_IN_LEN << 24) | SOF_block_storage_size(handle);
    cs.csum.for_info = checksum((uint8_t*)&cs.storage_len, 4);
    cs.csum.for_storage = compute_storage_checksum(handle);

    DPRINTF("write %#x at %#x"DCRLF,*((uint32_t*)&cs.csum), handle->storage_max_offset_);
    if (SOF_dev_write_word(handle->hdev_, handle->storage_max_offset_, *((uint32_t*)&cs.csum)) < 0)
        return false;

    if (SOF_dev_write_word(handle->hdev_, handle->storage_max_offset_+4, *((uint32_t*)&cs.storage_len)) < 0)
        return false;

    return true;
}

static bool create_empty_storage(SOF_DevHandle_t hdev, uint8_t sector_index)
{
    SOF_BlockHandle handle_data;

    handle_data.hdev_ = hdev;

    uint32_t info_begin_offset;

    if (!get_empty_info_location(&handle_data, &info_begin_offset)) {
        EPRINTF("no info"DCRLF);
        SOF_block_close(&handle_data);
        return false;
    }

    handle_data.storage_max_offset_ = info_begin_offset;
    handle_data.storage_begin_offset_ = 0;
    handle_data.storage_end_offset_ = handle_data.storage_begin_offset_;
    handle_data.cur_pos_ = handle_data.storage_begin_offset_;

    DPRINTF("storage created: begin=%d end=%d free=%d"DCRLF,
            handle_data.storage_begin_offset_, handle_data.storage_end_offset_, SOF_block_get_free_size(&handle_data));

    write_storage_info(&handle_data);

    return true;
}


bool SOF_block_format(uint8_t sector_index)
{
    if (!SOF_dev_is_valid_sector(sector_index)) {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE) {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }

    DPRINTF("Flash erase %d"DCRLF, sector_index);
    SOF_dev_erase(hdev);
    create_empty_storage(hdev, sector_index);
    SOF_dev_close(hdev);

    return true;
}

SOF_BlockHandle_t SOF_block_open_storage(uint8_t sector_index, SOF_Error_t *err)
{
    if (!SOF_dev_is_valid_sector(sector_index)) {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE) {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }

    SOF_BlockHandle_t handle = new SOF_BlockHandle();

    handle->hdev_ = hdev;

    StorageInfo_t storage_info;

    if ((*err=probe_active_storage_info(handle, &storage_info)) != kSOF_ErrNone) {
        delete handle;
        return NULL;
    }

    uint32_t info_begin_offset;

    if (!get_empty_info_location(handle, &info_begin_offset)) {
        *err = kSOF_ErrBadBlock;
        delete handle;
        return NULL;
    }

    // set max offset that storage grows.
    handle->storage_max_offset_ = info_begin_offset;

    handle->storage_begin_offset_ = storage_info.begin_offset;
    handle->storage_end_offset_ = storage_info.begin_offset + storage_info.len;

    handle->cur_pos_ = handle->storage_begin_offset_;

    DPRINTF("open for read: begin=%d end=%d len=%d free=%d"DCRLF,
            handle->storage_begin_offset_, handle->storage_end_offset_, storage_info.len,
            SOF_block_get_free_size(handle));
    if (compute_storage_checksum(handle) != storage_info.storage_csum) {
        EPRINTF("checksum error %#x != %#x"DCRLF, compute_storage_checksum(handle), storage_info.storage_csum);
        *err = kSOF_ErrDataCurrupted;
        delete handle;
        return NULL;
    }

    DUMP_BLOCK(handle);
    *err = kSOF_ErrNone;

    return handle;
}

SOF_BlockHandle_t SOF_block_create_storage(uint8_t sector_index, SOF_Error_t *err)
{
    if (!SOF_dev_is_valid_sector(sector_index)) {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return false;
    }

    SOF_DevHandle_t hdev = SOF_dev_open(sector_index);

    if (hdev == SOF_INVALID_HANDLE) {
        DPRINTF("SOF_dev_open(%d) failed"DCRLF, sector_index);
        return false;
    }

    SOF_BlockHandle_t handle = new SOF_BlockHandle();

    handle->hdev_ = hdev;

    StorageInfo_t storage_info;

    if ((*err=probe_active_storage_info(handle, &storage_info)) != kSOF_ErrNone) {
        delete handle;
        return NULL;
    }

    uint32_t info_begin_offset;

    if (!get_empty_info_location(handle, &info_begin_offset)) {
        *err = kSOF_ErrBadBlock;
        delete handle;
        return NULL;
    }

    // set max offset that storage grows.
    handle->storage_max_offset_ = info_begin_offset;

    // writing position is just after previous storage
    handle->storage_begin_offset_ = storage_info.begin_offset + storage_info.len;
    handle->storage_end_offset_ = handle->storage_begin_offset_;

    handle->cur_pos_ = handle->storage_begin_offset_;
    handle->write_mode_ = true;
    DPRINTF("open for write: begin=%d end=%d free=%d"DCRLF,
            handle->storage_begin_offset_, handle->storage_end_offset_, SOF_block_get_free_size(handle));

    DUMP_BLOCK(handle);
    *err = kSOF_ErrNone;

    return handle;
}

bool SOF_block_close(SOF_BlockHandle_t handle)
{
    bool r = true;

    DASSERT(handle != NULL);
    if (handle->write_mode_)
        r = (bool)write_storage_info(handle);
    SOF_dev_close(handle->hdev_);
    delete handle;

    return r;
}

uint8_t *SOF_block_base_addr(SOF_BlockHandle_t handle)
{
    DASSERT(handle != NULL);
    return SOF_dev_get_hw_addr(handle->hdev_) + handle->cur_pos_;
}

bool SOF_block_putc(SOF_BlockHandle_t handle, uint8_t c)
{
    DASSERT(handle != NULL);
    DASSERT(handle->is_writable());

    if (SOF_block_get_free_size(handle) == 0) {
        DPRINTF("no free space"DCRLF);
        DUMP_BLOCK(handle);

        return false;
    }

    bool b = SOF_dev_write_byte(handle->hdev_, handle->cur_pos_, c) != -1;
    if (b) {
        handle->cur_pos_++;
        handle->storage_end_offset_++;
    }

    return b;
}

size_t SOF_block_write(SOF_BlockHandle_t handle, const uint8_t *p, size_t p_size)
{
    size_t i;

    for (i = 0; i < p_size; ++i)
        if (SOF_block_putc(handle, *p++) != true)
            return i;

    return i;
}

bool SOF_block_getc(SOF_BlockHandle_t handle, uint8_t *c)
{
    DASSERT(handle != NULL);
    DASSERT(handle->is_writable());

    if (handle->cur_pos_ >= handle->storage_end_offset_) {
        DPRINTF("end of data\n"DCRLF);
        DUMP_BLOCK(handle);

        return false;
    }

    *c = SOF_dev_read_byte(handle->hdev_, handle->cur_pos_++);

    return true;
}

size_t SOF_block_read(SOF_BlockHandle_t handle, uint8_t *p, size_t p_size)
{
    size_t i;

    for (i = 0; i < p_size; ++i)
        if (!SOF_block_getc(handle, p++))
            break;

    return i;
}

SOF_Error_t SOF_block_get_statics(uint8_t sector_index, SOF_Statics_t *stat)
{
    if (!SOF_dev_is_valid_sector(sector_index)) {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return kSOF_ErrParam;
    }

    SOF_Error_t err;
    SOF_BlockHandle_t hblk = SOF_block_open_storage(sector_index, &err);

    if (hblk == NULL) {
        DPRINTF("SOF_block_open_storage(%d) failed"DCRLF, sector_index);
        return err;
    }

    stat->data_addr = SOF_block_base_addr(hblk);
    stat->data_size = SOF_block_storage_size(hblk);
    stat->free_size = SOF_block_get_free_size(hblk);

    SOF_block_close(hblk);

    return kSOF_ErrNone;
}

const SOF_SectorSpec_t *SOF_block_get_info(uint8_t sector_index)
{
    if (!SOF_dev_is_valid_sector(sector_index)) {
        DPRINTF("invalid sector_index=%d"DCRLF, sector_index);
        return NULL;
    }

    return SOF_dev_info_by_index(sector_index);
}


