/*
* @file SOFBlock.cpp
* @author hillkim7@gmail.com
*
* @brief Simple storage implementation on internal MCU flash memory.
*
*/

#include "SOFBlock.h"
#include <string.h>
#include <stdlib.h>


SOFBlock::SOFBlock()
    : hblock_(NULL)
{
}

SOFBlock::~SOFBlock()
{
    close();
}

void SOFBlock::close()
{
    if (hblock_ != NULL) {
        SOF_block_close(hblock_);
        hblock_ = NULL;
    }
}

bool SOFBlock::format( uint8_t sector_index )
{
    return SOF_block_format(sector_index);
}

bool SOFBlock::get_stat( uint8_t sector_index, SOF_Statics_t &statics )
{
    return SOF_block_get_statics(sector_index, &statics) ==  kSOF_ErrNone;
}

SOFWriter::SOFWriter()
{

}

SOFWriter::~SOFWriter()
{

}

SOF_Error_t SOFWriter::open( uint8_t sector_index )
{
    if (is_open()) {
        return kSOF_ErrBusyBlock;
    }

    SOF_Error_t err;

    hblock_ = SOF_block_create_storage(sector_index, &err);

    return err;
}

bool SOFWriter::write_byte_data( uint8_t c )
{
    if (!is_open()) {
        return false;
    }

    return SOF_block_putc(hblock_, c);
}

size_t SOFWriter::write_data( const uint8_t *p, size_t p_size )
{
    if (!is_open()) {
        return false;
    }

    return SOF_block_write(hblock_, p, p_size);
}

size_t SOFWriter::get_free_size()
{
    if (!is_open()) {
        return 0;
    }

    return SOF_block_get_free_size(hblock_);
}


SOFReader::SOFReader()
{

}

SOFReader::~SOFReader()
{

}

SOF_Error_t SOFReader::open( uint8_t sector_index )
{
    if (is_open()) {
        return kSOF_ErrBusyBlock;
    }

    SOF_Error_t err;

    hblock_ = SOF_block_open_storage(sector_index, &err);

    return err;
}

uint8_t * SOFReader::get_physical_data_addr()
{
    if (!is_open()) {
        return NULL;
    }

    return SOF_block_base_addr(hblock_);
}

size_t SOFReader::get_data_size()
{
    if (!is_open()) {
        return 0;
    }

    return SOF_block_storage_size(hblock_);
}

bool SOFReader::read_byte_data( uint8_t *c )
{
    if (!is_open()) {
        return false;
    }

    return SOF_block_getc(hblock_, c);
}

size_t SOFReader::read_data( uint8_t *p, size_t p_size )
{
    if (!is_open()) {
        return 0;
    }

    return SOF_block_read(hblock_, p, p_size);
}


