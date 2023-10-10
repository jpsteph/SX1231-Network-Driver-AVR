

#include <avr/io.h>
#include "spi.h"

void spi_init()
// Initialize pins for spi communication
{
    SPI_DDR &= ~((1<<MOSI)|(1<<MISO)|(1<<SS)|(1<<SCK));
    SPI_DDR |= ((1<<MOSI)|(1<<SS)|(1<<SCK));

    
    SPCR = ((1<<SPE)|(0<<SPIE)|(0<<DORD)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0)|(0<<CPOL)|(0<<CPHA));             
    SPSR = (1<<SPI2X);              
    
}

void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        SPDR = dataout[i];
        while((SPSR & (1<<SPIF))==0);
        datain[i] = SPDR;
    }
}

void spi_transmit_sync(uint8_t * dataout, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        SPDR = dataout[i];
        while((SPSR & (1<<SPIF))==0);
    }
}

uint8_t spi_fast_shift(uint8_t data)
{
    SPDR = data;
    while((SPSR & (1<<SPIF))==0);
    return SPDR;
}
