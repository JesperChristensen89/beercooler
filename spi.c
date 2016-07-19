//-------------------------------------------------------------
// include
//-------------------------------------------------------------
#include "spi.h"

//-------------------------------------------------------------
// functions
//-------------------------------------------------------------
void SPI_master_init(void) {
  
  DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS); // Set MOSI, SCK and SS to output, all others input
  SPCR    |= (1 << SPE) | (1 << MSTR); // Enable SPI, master mode, set clock rate fck/4 (SPSR lsb is 0)
  //SPSR    |= (1 << SPI2X); // Double SPI Speed Bit
  
}

void SPI_slave_init(void) {
  
  DDR_SPI |= (1 << DD_MISO); // Set MISO to output, all others input
  SPCR    |= (1 << SPE); // Enable SPI, slave mode
  
}

uint8_t SPI_write_byte(uint8_t data) {
  
  SPDR = data; // write data to transmit register to start transmission
  while (!(SPSR & (1 << SPIF))); // wait for transmission complete
  return SPDR; // return data recived during transfer

}

uint8_t SPI_read_byte(void) {
    
  while (!(SPSR & (1 << SPIF))); // wait for transmission complete
  return SPDR; // return data recived during transfer

}
