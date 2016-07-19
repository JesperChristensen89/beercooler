//-------------------------------------------------------------
// include
//-------------------------------------------------------------
#include <avr/io.h>
#include <inttypes.h>

//-------------------------------------------------------------
// define
//-------------------------------------------------------------
#define DDR_SPI  DDRB
#define PORT_SPI PORTB
#define DD_MISO  PB4
#define DD_MOSI  PB3
#define DD_SCK   PB5
#define DD_SS    PB2

//-------------------------------------------------------------
// function prototypes
//-------------------------------------------------------------
void SPI_master_init(void);
void SPI_slave_init(void);
uint8_t SPI_write_byte(uint8_t data);
uint8_t SPI_read_byte(void);
