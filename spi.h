#ifndef _TN84_SPI_H_
#define _TN84_SPI_H_

#include <avr/io.h>

// NOTE The following PORTA PINs are hardcoded (!)
// PA4 - SCK
// PA5 - MISO
// PA6 - MOSI

void spi_init()
{
  DDRA |= _BV(DDA4) | _BV(DDA5);
  DDRA &= ~_BV(DDA6);

  PORTA |= _BV(PA6);
  PORTA &= ~(_BV(PA4) | _BV(PA5));
}

uint8_t spi_transfer(uint8_t data)
{
  USIDR = data;
  USISR = _BV(USIOIF);

  while ((USISR & _BV(USIOIF)) == 0)
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);

  return USIDR;
}

#endif
