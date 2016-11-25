/**
 * @file SOFBlock.h
 *
 * @author hillkim7@gmail.com
 * @brief Simple storage implementation on internal MCU flash memory.
 *
 * The SOF in SOFBlock is abbreviation of "Storage On Flash".
 * The purpose of SOFBlock class is to provide a way to write data on flash memory
 * in the same way of file handling class in the file system.
 * It manages a chunk of data on the Flash memory efficiently by minimizing flash erase operation as little as possible.
 * Note: Currently it only supports STM32F4xx series platforms.
 *       - NUCLEO-F401RE, NUCLEO-F411RE, Seeed Arch Max
 * The STM32 F4xx series from ST have plenty of internal Flash memory inside MCU core.
 * For example STM32 401RE has 512Kbyts Flash.
 * Typical size of firmware file is less than 256KB, so remaining area is free to use.
 * The simplest way of flash utilization as data storage is to use a chunk of Flash area as an unit of storage.
 * A block of flash is called sector in STM32F4xx domain. It requires to erase a sector before update bits in flash.
 *
 * Conceptually it is quite simple.
 * Here is typical write operation:
 * 1) Erase sector #n
 * 2) Write data to sector #n
 * Read operation:
 * 1) Just read physical memory address of sector #n
 *    The base physical address of STM32 flash is 0x08000000.
 *
 * There may be inefficiency in this flash usage scenario when size of data is too small compared with sector size.
 * The size of sectors from #5 to #7 of STM32-F4xx Flash is 128KB. For example, if I only need to maintain 1KB data,
 * whenever I need to update data I need to erase whole 128KB of sector.
 * This produces two problems.
 * One is time consumption of the erase operation. The operation of ERASE128KB takes 1~4 seconds long.
 * The other is related to lifetime of Flash memory.
 * More you erase and write and lifetime of flash is shorter.
 *
 * To overcome such problems, here simple flash management algorithm is used for.
 * By tracking data offset and size it can hold multiple data in a sector.
 * Bear in mind that is impossible rewriting data on Flash.
 * Keeping tracking data along with data itself without frequent erase operation is crucial.
 * To do this, data itself is growing from low address.
 * On the other hand tracking data is growing down from high address.
 * Let's assume the size of data is 1KB and store it in sector #6 which address range is from 0x08040000 to 0x0805ffff.
 * +-------------+----------------------------------------------------------------------+-----+
 * <data>                                                                        <tracking data>
 * +-------------+----------------------------------------------------------------------+-----+
 * data grows ->                                                           <- tracking data grows
 * Writing data will be placed at the end of data always and reading data will pick the last data.
 * It is like simple file system that only keep a file only.
 *
 * Unlike file manipulation operation, there is caution you need to check if write operation fails
 * or need to check free size before you start to write data.
 * It is required to format flash sector when there is no more free space.
 */

#pragma once

#include "SOF_dev.h"

/** SOF(Storage On Flash) usage example
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "SOFBlock.h"
 *
 * int main()
 * {
 * 	const uint8_t sector_index = 7;
 * 	SOFBlock::format(sector_index);	// Erase flash sector 7 and make structure for storage.
 *
 * 	SOFWriter writer;
 * 	SOFReader reader;
 *
 * 	writer.open(sector_index);
 * 	writer.write_data((uint8_t*)"First Data", 10);
 * 	writer.close();
 *
 * 	reader.open(sector_index);
 * 	printf("data %d bytes at %p :\r\n", reader.get_data_size(), reader.get_physical_base_addr());
 * 	printf("%.*s\r\n", reader.get_data_size(), reader.get_physical_base_addr());
 * 	// "First Data" printed
 * 	reader.close();
 *
 *  SOF_Statics_t statics;
 *  if (!SOFBlock(sector_index, statics) || statics.free_size < 11) { // check available byte
 *     SOFBlock::format(sector_index);
 *  }
 * 	writer.open(sector_index);
 * 	// Overwrite previous data without erasing flash.
 * 	writer.write_data((uint8_t*)"Second Data", 11);
 * 	writer.close();
 *
 * 	reader.open(sector_index);
 * 	printf("data %d bytes at %p :\r\n", reader.get_data_size(), reader.get_physical_base_addr());
 * 	printf("%.*s\r\n", reader.get_data_size(), reader.get_physical_base_addr());
 * 	// "Second Data" printed
 * 	reader.close();
 * }
 */

/**
 * Base class of SOF(Storage On Flash)
 */
class SOFBlock
{
public:
    SOFBlock();

    virtual ~SOFBlock();

    void close();

public:
    /*** Returns whether instance of SOFBlock is currently associated to flash storage.  */
    bool is_open() const {
        return hblock_ != NULL;
    }

public:
    /*** Erase flash sector and put signature to setup file system struct */
    static bool format(uint8_t sector_index);

    /*** Get statistics of storage */
    static bool get_stat(uint8_t sector_index, SOF_Statics_t &statics);

protected:
    SOF_BlockHandle_t hblock_;
};


/**
 * It provides interface for writing data to flash memory.
 */
class SOFWriter : public SOFBlock
{
public:
    SOFWriter();
    virtual ~SOFWriter();

    /*** Open for writing mode */
    SOF_Error_t open(uint8_t sector_index);

    /*** Return max available for writing */
    size_t get_free_size();

    /*** Write one byte of data.
     * Note: in case of storage full, it can't write data any more.
     * It is required to format sector and write it again.
     */
    bool write_byte_data(uint8_t c);

    /*** Write n bytes of data */
    size_t write_data(const uint8_t *p, size_t p_size);
};


/**
* It provides interface for reading data from flash memory.
* It can read data directly by accessing physical flash address or
* calling function like traditional file API style.
*/
class SOFReader : public SOFBlock
{
public:
    SOFReader();
    virtual ~SOFReader();

    /*** Open for read mode */
    SOF_Error_t open(uint8_t sector_index);

    /*** Return flash physical address of data for direct access */
    uint8_t *get_physical_data_addr();

    /*** Return data size */
    size_t get_data_size();

    /*** Return one byte of data */
    bool read_byte_data(uint8_t *c);

    /*** Return n bytes of data */
    size_t read_data( uint8_t *p, size_t p_size);
};


