#ifndef __nand_spi_flash_included__
#define __nand_spi_flash_included__

#define NAND_SPI_FLASH_STR_ERROR

#include <stdint.h>

// Nand Flash Device Specific
// page size in bytes
#define NAND_FLASH_PER_PAGE_SIZE            2112

// block size in pages
#define NAND_FLASH_PAGE_COUNT_PER_BLOCK     128

// blocks count in memory
#define NAND_FLASH_BLOCK_COUNT              2048

// total row count
#define NAND_FLASH_ROW_COUNT                NAND_FLASH_PAGE_COUNT_PER_BLOCK * NAND_FLASH_BLOCK_COUNT

// Possible error codes, these are negative to allow
// valid positive return values
enum nand_spi_flash_error {
    NSF_ERR_OK              =  0,    // No error
    NSF_ERR_NOT_INITED      = -1,    // Nand SPI Flash driver not inited
    NSF_ERR_ALREADY_INITED  = -2,    // Nand SPI Flash driver already inited
    NSF_ERR_UNKNOWN_DEVICE  = -3,    // Nand SPI Flash unsupported device
    NSF_ERR_READ_ONLY       = -4,    // Device read-only
    NSF_ERR_BAD_BLOCK       = -5,    // Device read-only
    NSF_ERR_DATA_TOO_BIG    = -6,    // Data to write/read is greater than page
    NSF_ERR_ERASE           = -7,    // Block erase hardware error
    NSF_ERR_PROGRAM         = -8,    // Block write hardware error
    NSF_ERROR_SPI           = -100   // Error 
};

#ifdef NAND_SPI_FLASH_STR_ERROR
// return error description
const char * nand_spi_flash_str_error(int error);
#endif 

// Nand Spi flash size
uint16_t nand_spi_flash_page_size_bytes(void);

// Nand Spi Flash block size
uint16_t nand_spi_flash_block_size_pages(void);

// Nand Spi Flash blocks count
uint16_t nand_spi_flash_blocks_count(void);

// Init driver
// @returns 0 on success, negative error code on error
int nand_spi_flash_init(void);

// Reset and unlock flash device
int nand_spi_flash_reset_unlock(void);

// Read Nand Flash Status Byte
uint8_t nand_spi_flash_read_status(void);

// Page read
// @row_address is block_address (first 18 bits) + page_address (6 bits)
// @col_address is the byte address in the page
// @returns number of bytes read or negative error code on error
int nand_spi_flash_page_read(uint32_t row_address, uint16_t col_address, uint8_t * data, uint16_t read_len);

int nand_spi_flash_write_enable(void);

// Page write
// @row_address is block_address (first 18 bits) + page_address (6 bits)
// @col_address is the byte address in the page
// @returns number of bytes written or negative error code on error
int nand_spi_flash_page_write(uint32_t row_address, uint16_t col_address, uint8_t * data, uint16_t data_len);

// Block erase
int nand_spi_flash_block_erase(uint32_t row_address);

#endif //__nand_spi_flash_included__
