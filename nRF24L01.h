#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "spi.h"

////////////////////////////////////////////////////////////////////////////////
// The following must be defined beforehand!
// Below are configuration examples only!
//
// NOTE PORTA PINs are hardcoded (!)
//
// #define NRF_CFG_CE_PIN PA2
// #define NRF_CFG_CSN_PIN PA3
// #define NRF_CFG_CHANNEL 1
//
// #define NRF_CFG_CONFIG 0x0C // 16bit CRC
// #define NRF_CFG_RF_SETUP 0x0E // 2Mbps 0dBm
// #define NRF_CFG_SETUP_RETR 0x3F // ARD: 0x3 (1000 us), ARC: 0xF (15x)
//
// uint8_t nrf_rx_addr[] = { 0x01, 0xAA, 0xAA, 0xAA, 0xAA };
// uint8_t nrf_tx_addr[] = { 0x00, 0xAA, 0xAA, 0xAA, 0xAA };
//
////////////////////////////////////////////////////////////////////////////////

#define NRF_ADDRESS_LENGTH (sizeof(nrf_rx_addr) / sizeof(nrf_rx_addr[0]))
#define NRF_MAX_PAYLOAD_LENGTH 32

// REGISTERS
#define NRF_CONFIG 0x00
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_RX_ADDR_P1 0x0B
#define NRF_TX_ADDR 0x10
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD 0x1C
#define NRF_FEATURE 0x1D

// REGISTER BITS
#define NRF_PWR_UP 1
#define NRF_PRIM_RX 0
#define NRF_RX_DR 6
#define NRF_TX_DS 5
#define NRF_MAX_RT 4
#define NRF_RX_EMPTY 0
#define NRF_EN_DPL 2
#define NRF_EN_ACK_PAY 1

// COMMANDS
#define NRF_R_REGISTER 0x00
#define NRF_W_REGISTER 0x20
#define NRF_REGISTER_MASK 0x1F
#define NRF_R_RX_PL_WID 0x60
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_W_TX_PAYLOAD 0xA0
#define NRF_FLUSH_TX 0xE1
#define NRF_FLUSH_RX 0xE2
#define NRF_NOP 0xFF

uint8_t nrf_ptx __attribute__ ((section (".noinit")));

void nrf_init();
uint8_t nrf_data_ready();
void nrf_get_data(uint8_t *data, uint8_t *len);
void nrf_send_data(uint8_t *data, uint8_t len);
uint8_t nrf_sending(uint8_t *status);
uint8_t nrf_sent_ok(uint8_t status);

void nrf_ce_hi();
void nrf_ce_low();
void nrf_csn_hi();
void nrf_csn_low();
void nrf_config_register(uint8_t reg, uint8_t value);
void nrf_read_register(uint8_t reg, uint8_t *value, uint8_t len);
void nrf_write_register(uint8_t reg, uint8_t *value, uint8_t len);
uint8_t nrf_get_status();
void nrf_flush_tx();
void nrf_flush_rx();
uint8_t nrf_rx_fifo_empty();
void nrf_power_up_rx(uint8_t flush);
void nrf_power_up_tx();
void nrf_spi_write(uint8_t reg, uint8_t *data, uint8_t read_data, uint8_t len);
void nrf_power_down();

void nrf_ce_hi()
{
  PORTA |= _BV(NRF_CFG_CE_PIN);
}

void nrf_ce_low()
{
  PORTA &= ~_BV(NRF_CFG_CE_PIN);
}

void nrf_csn_hi()
{
  PORTA |= _BV(NRF_CFG_CSN_PIN);
}

void nrf_csn_low()
{
  PORTA &= ~_BV(NRF_CFG_CSN_PIN);
}

void nrf_config_register(uint8_t reg, uint8_t value)
{
  nrf_write_register(reg, &value, 1);
}

void nrf_read_register(uint8_t reg, uint8_t *value, uint8_t len)
{
  nrf_spi_write((NRF_R_REGISTER | (NRF_REGISTER_MASK & reg)), value, 1, len);
}

void nrf_write_register(uint8_t reg, uint8_t *value, uint8_t len)
{
  nrf_spi_write((NRF_W_REGISTER | (NRF_REGISTER_MASK & reg)), value, 0, len);
}

uint8_t nrf_get_status()
{
  uint8_t rv = NRF_NOP;
  nrf_read_register(NRF_STATUS, &rv, 1);
  return rv;
}

void nrf_flush_tx()
{
  nrf_spi_write(NRF_FLUSH_TX, 0, 0, 0);
}

void nrf_flush_rx()
{
  nrf_spi_write(NRF_FLUSH_RX, 0, 0, 0);
}

void nrf_init()
{
  nrf_ptx = 0;

  DDRA |= _BV(NRF_CFG_CE_PIN) | _BV(NRF_CFG_CSN_PIN);

  nrf_ce_low();
  nrf_csn_hi();

  spi_init();

  nrf_config_register(NRF_RF_SETUP, NRF_CFG_RF_SETUP);
  nrf_config_register(NRF_RF_CH, NRF_CFG_CHANNEL);

  nrf_config_register(NRF_SETUP_RETR, NRF_CFG_SETUP_RETR);

  // Enable only ERX_P0 - NRF_RX_ADDR_P0
  nrf_config_register(NRF_EN_RXADDR, 0x01);

  nrf_write_register(NRF_RX_ADDR_P1, nrf_rx_addr, NRF_ADDRESS_LENGTH);
  nrf_write_register(NRF_RX_ADDR_P0, nrf_tx_addr, NRF_ADDRESS_LENGTH);
  nrf_write_register(NRF_TX_ADDR, nrf_tx_addr, NRF_ADDRESS_LENGTH);

  // Enable dynamic payload and ACK payload for P0.
  nrf_config_register(NRF_FEATURE, _BV(NRF_EN_DPL) | _BV(NRF_EN_ACK_PAY));
  nrf_config_register(NRF_DYNPD, 0x03);

  nrf_power_up_rx(1);
  nrf_flush_rx();
}

uint8_t nrf_data_ready()
{
  uint8_t status = nrf_get_status();

  if (status & _BV(NRF_RX_DR))
    return 1;

  return !nrf_rx_fifo_empty();
}

uint8_t nrf_rx_fifo_empty()
{
  uint8_t fifo_status;

  nrf_read_register(NRF_FIFO_STATUS, &fifo_status, sizeof(fifo_status));

  return fifo_status & _BV(NRF_RX_EMPTY);
}

void nrf_get_data(uint8_t *data, uint8_t *len)
{
  nrf_spi_write(NRF_R_RX_PL_WID, len, 1, 1);
  nrf_spi_write(NRF_R_RX_PAYLOAD, data, 1, *len);
  nrf_config_register(NRF_STATUS, _BV(NRF_RX_DR));
}

void nrf_send_data(uint8_t *data, uint8_t len)
{
  nrf_ce_low();
  nrf_power_up_tx();
  nrf_flush_tx();

  nrf_spi_write(NRF_W_TX_PAYLOAD, data, 0, len);

  nrf_ce_hi();
  nrf_ce_low();
}

uint8_t nrf_sending(uint8_t *status)
{
  if (nrf_ptx)
  {
    *status = nrf_get_status();

    if (*status & (_BV(NRF_TX_DS) | _BV(NRF_MAX_RT)))
    {
      nrf_power_up_rx(0);
      return 0;
    }

    return 1;
  }

  return 0;
}

uint8_t nrf_sent_ok(uint8_t status)
{
  return !(status & _BV(NRF_MAX_RT));
}

void nrf_power_up_rx(uint8_t flush)
{
  nrf_ptx = 0;
  nrf_ce_low();
  nrf_config_register(NRF_CONFIG, NRF_CFG_CONFIG | _BV(NRF_PWR_UP) | _BV(NRF_PRIM_RX));

  if (flush)
    nrf_config_register(NRF_STATUS, _BV(NRF_RX_DR) | _BV(NRF_TX_DS) | _BV(NRF_MAX_RT));

  nrf_ce_hi();
}

void nrf_power_up_tx()
{
  nrf_ptx = 1;
  nrf_config_register(NRF_CONFIG, NRF_CFG_CONFIG | (_BV(NRF_PWR_UP) & ~_BV(NRF_PRIM_RX)));
}

void nrf_spi_write(uint8_t reg, uint8_t *data, uint8_t read_data, uint8_t len)
{
  nrf_csn_low();
  spi_transfer(reg);

  if (data)
  {
    uint8_t i;

    for (i = 0; i < len; ++i)
    {
      uint8_t read_value = spi_transfer(data[i]);

      if (read_data)
        data[i] = read_value;
    }
  }

  nrf_csn_hi();
}

void nrf_power_down()
{
  nrf_ce_low();
  nrf_config_register(NRF_CONFIG, NRF_CFG_CONFIG);
}

#endif
