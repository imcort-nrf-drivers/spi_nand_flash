#include "nand_spi_flash.h"
#include "transfer_handler.h"

// Nand Flash Commands
#define NSF_CMD_MAX_BYTES 4

#define NSF_CMD_READ_ID 0x9F
#define NSF_CMD_READ_CELL_TO_CACHE 0x13
#define NSF_CMD_GET_FEATURE 0x0F
#define NSF_CMD_SET_FEATURE 0x1F
#define NSF_CMD_FEATURE_STATUS 0xC0
#define NSF_CMD_FEATURE_LOCK 0xA0
#define NSF_CMD_RESET 0xFF
#define NSF_CMD_WRITE_ENABLE 0x06
#define NSF_CMD_BLOCK_ERASE 0xD8
#define NSF_CMD_PROGRAM_LOAD 0x02
#define NSF_CMD_PROGRAM_LOAD_RANDOM 0x84
#define NSF_CMD_PROGRAM_EXECUTE 0x10

// Nand Flash Status Bits
#define NSF_OIP_MASK 0x01
#define NSF_PRG_F_MASK 0x08 //0b00001000
#define NSF_ERS_F_MASK 0x04 //0b00000100
#define NSF_ECC_MASK 0x30   //0b00110000
#define NSF_ECC_BITS 4

// Timings
#define NSF_PAGE_READ_TIME_US 115
#define NSF_RESET_TIME_MS 7

// spi read/write buffer
static uint8_t m_nsf_buffer[255];

// Which page data in nand cache, default is none.
static uint32_t read_row_addr_in_cache = NAND_FLASH_ROW_COUNT;

static int nand_spi_transfer(uint8_t *buffer, uint16_t tx_len, uint16_t rx_len)
{
	digitalWrite(SPI_NAND_FLASH_CS, LOW);
    Debug("nand_spi_transfer size: TX: %d, RX: %d", tx_len, rx_len);
    spi_transfer(buffer, tx_len, buffer, tx_len + rx_len);
	digitalWrite(SPI_NAND_FLASH_CS, HIGH);
		
    return NSF_ERR_OK;
}

//-----------------------------------------------------------------------------
int nand_spi_flash_reset_unlock()
{
    // reset device
    m_nsf_buffer[0] = NSF_CMD_RESET;
    if (nand_spi_transfer(m_nsf_buffer, 1, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }
    nand_spi_flash_read_status();

    // unlock blocks for write
    m_nsf_buffer[0] = NSF_CMD_SET_FEATURE;
    m_nsf_buffer[1] = NSF_CMD_FEATURE_LOCK;
    m_nsf_buffer[2] = 0x00;
    if (nand_spi_transfer(m_nsf_buffer, 3, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }

    return NSF_ERR_OK;
}

//-----------------------------------------------------------------------------
int nand_spi_flash_write_enable(void)
{
    
    // write enable
    m_nsf_buffer[0] = NSF_CMD_WRITE_ENABLE;
    if (nand_spi_transfer(m_nsf_buffer, 1, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }
    return NSF_ERR_OK;
		
}

//-----------------------------------------------------------------------------
int nand_spi_flash_init(void)
{
  // check spi driver already inited and copy config
    pinMode(SPI_NAND_FLASH_CS, OUTPUT);
	digitalWrite(SPI_NAND_FLASH_CS, HIGH);
    
    spi_init();
	
	m_nsf_buffer[0] = NSF_CMD_READ_ID;
	m_nsf_buffer[1] = 0x00;
    if (nand_spi_transfer(m_nsf_buffer, 2, 2) != 0)
    {
        return NSF_ERROR_SPI;
    }
	Debug("ID0 %x, ID1 %x", m_nsf_buffer[2], m_nsf_buffer[3]);

//    //Disable HSE Mode
//    m_nsf_buffer[0] = 0x1F;
//    m_nsf_buffer[1] = 0xB0;
//    m_nsf_buffer[2] = 0x00;
    
    nand_spi_flash_reset_unlock();
    return nand_spi_flash_write_enable();

}

//-----------------------------------------------------------------------------
uint8_t nand_spi_flash_read_status()
{
    m_nsf_buffer[2] = NSF_OIP_MASK;          //Operation in progress
    while (m_nsf_buffer[2] & NSF_OIP_MASK)
    {
        nrf_delay_us(NSF_PAGE_READ_TIME_US);
        m_nsf_buffer[0] = NSF_CMD_GET_FEATURE;
        m_nsf_buffer[1] = NSF_CMD_FEATURE_STATUS;
        nand_spi_transfer(m_nsf_buffer, 2, 1);
    }
    return m_nsf_buffer[2];
}

//-----------------------------------------------------------------------------
int nand_spi_flash_page_read(uint32_t row_address, uint16_t col_address, uint8_t *buffer, uint16_t read_len)
{
    // check data len
    if ((read_len + col_address) >= NAND_FLASH_PER_PAGE_SIZE)   return NSF_ERR_DATA_TOO_BIG;
    if (row_address >= NAND_FLASH_ROW_COUNT)                    return NSF_ERR_DATA_TOO_BIG;
    
    // check if it is stored
    if (read_row_addr_in_cache != row_address)
    {
        // read page to nand cache buffer
        m_nsf_buffer[0] = NSF_CMD_READ_CELL_TO_CACHE;
        m_nsf_buffer[1] = (row_address & 0xff0000) >> 16;
        m_nsf_buffer[2] = (row_address & 0xff00) >> 8;
        m_nsf_buffer[3] = row_address; // & 0xff;
        
        if (nand_spi_transfer(m_nsf_buffer, 4, 0) != 0)
        {
            return NSF_ERROR_SPI;
        }

        // check status
        if ((nand_spi_flash_read_status() & NSF_ECC_MASK) == NSF_ECC_MASK)
        {
            return NSF_ERR_BAD_BLOCK;
        }    
        
        read_row_addr_in_cache = row_address;
        
    }
    
    // read buffer from cache
    m_nsf_buffer[0] = 0x03;
    m_nsf_buffer[1] = (col_address & 0xff00) >> 8;
    m_nsf_buffer[2] = col_address; // & 0xff;
    m_nsf_buffer[3] = 0x00;
    
    if (nand_spi_transfer(m_nsf_buffer, 4, read_len) != 0)
    {
        return NSF_ERROR_SPI;
    }

    // copy data to output buffer
    memcpy(buffer, &m_nsf_buffer[4], read_len);

    return read_len;
    
}



//-----------------------------------------------------------------------------
int nand_spi_flash_page_write(uint32_t row_address, uint16_t col_address, uint8_t *data, uint16_t data_len)
{
    // check data len
    if ((data_len + col_address) >= NAND_FLASH_PER_PAGE_SIZE)   return NSF_ERR_DATA_TOO_BIG;
    if (row_address >= NAND_FLASH_ROW_COUNT)                    return NSF_ERR_DATA_TOO_BIG;
    
    // copy buffer to nand cache
    m_nsf_buffer[0] = NSF_CMD_PROGRAM_LOAD;
    m_nsf_buffer[1] = (col_address & 0xff00) >> 8;
    m_nsf_buffer[2] = col_address; // & 0xff;
    
    memcpy(&m_nsf_buffer[3], data, data_len);
    if (nand_spi_transfer(m_nsf_buffer, data_len + 3, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }
    
    read_row_addr_in_cache = NAND_FLASH_ROW_COUNT;  //Cache cleared.
    
    // program execute
    m_nsf_buffer[0] = NSF_CMD_PROGRAM_EXECUTE;
    m_nsf_buffer[1] = (row_address & 0xff0000) >> 16;
    m_nsf_buffer[2] = (row_address & 0xff00) >> 8;
    m_nsf_buffer[3] = row_address; // & 0xff;
    if (nand_spi_transfer(m_nsf_buffer, 4, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }
    
    // check status
    if ((nand_spi_flash_read_status() & NSF_ECC_MASK) == NSF_ECC_MASK)
    {
        return NSF_ERR_BAD_BLOCK;
    } 
    
	return data_len;
    
}



//-----------------------------------------------------------------------------
int nand_spi_flash_block_erase(uint32_t row_address)
{
    // enable write
    m_nsf_buffer[0] = NSF_CMD_WRITE_ENABLE;
    if (nand_spi_transfer(m_nsf_buffer, 1, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }

    // erase block
    m_nsf_buffer[0] = NSF_CMD_BLOCK_ERASE;
    m_nsf_buffer[1] = (row_address & 0xff0000) >> 16;
    m_nsf_buffer[2] = (row_address & 0xff00) >> 8;
    m_nsf_buffer[3] = row_address; // & 0xff;
    if (nand_spi_transfer(m_nsf_buffer, 4, 0) != 0)
    {
        return NSF_ERROR_SPI;
    }

    return (nand_spi_flash_read_status() & NSF_ERS_F_MASK) ? NSF_ERR_ERASE : NSF_ERR_OK;
    
}

#ifdef NAND_SPI_FLASH_STR_ERROR
//-----------------------------------------------------------------------------
const char *nand_spi_flash_str_error(int error)
{
  if (error >= 0)
  {
    return "NSF_ERR_OK";
  }
  switch (error)
  {
  case -1:
    return "NSF_ERR_NOT_INITED";
  case -2:
    return "NSF_ERR_ALREADY_INITED";
  case -3:
    return "NSF_ERR_UNKNOWN_DEVICE";
  case -4:
    return "NSF_ERR_READ_ONLY";
  case -5:
    return "NSF_ERR_BAD_BLOCK";
  case -6:
    return "NSF_ERR_DATA_TOO_BIG";
  case -7:
    return "NSF_ERR_ERASE";
  case -8:
    return "NSF_ERR_PROGRAM";
  case -100:
    return "NSF_ERROR_SPI";
  default:
    return "NSF_UNKNOWN_ERROR";
  }
}
#endif
