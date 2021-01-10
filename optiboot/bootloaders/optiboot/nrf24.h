/*
 * nRF24L01+ only (note the plus).
 *
 * Licensed under AGPLv3.
 */
#include "nRF24L01.h"
#include <inttypes.h>

#include <avr/io.h>

#warning Make sure pin config matches hardware setup.
#warning Here CE  = PIN9  (PORTB1)
#warning Here CSN = PIN10 (PORTB2)
#define CE_DDR		DDRB
#define CE_PORT		PORTB
#define CSN_DDR		DDRB
#define CSN_PORT	PORTB
#define CE_PIN		(1 << 1)
#define CSN_PIN		(1 << 2)

//Should be redundant. Want to avoid *and / of floats
#ifndef TIMER
//#define my_delay(msec) delay8((int) (F_CPU / 8000L * (msec)))
#define my_delay(usec) delay8((int) (F_CPU / 8000000L * (usec)))
#endif

/* Enable 16-bit CRC */
#define CONFIG_VAL ((1 << MASK_RX_DR) | (1 << MASK_TX_DS) | \
		(1 << MASK_MAX_RT) | (1 << CRCO) | (1 << EN_CRC))

//Should be redundant. Want to avoid *and / of floats
#ifndef TIMER
//#define my_delay(msec) delay8((int) (F_CPU / 8000L * (msec)))
#define my_delay(usec) delay8((int) (F_CPU / 8000000L * (usec)))
#endif

/* Enable 16-bit CRC */
#define CONFIG_VAL ((1 << MASK_RX_DR) | (1 << MASK_TX_DS) | \
		(1 << MASK_MAX_RT) | (1 << CRCO) | (1 << EN_CRC))

static uint8_t nrf24_in_rx = 0;

 void delay8(uint16_t count);
 void nrf24_ce(uint8_t level);
 void nrf24_csn(uint8_t level);
 uint8_t nrf24_read_reg(uint8_t addr);
 void nrf24_write_reg(uint8_t addr, uint8_t value) ;
uint8_t nrf24_read_status(void);
void nrf24_write_addr_reg(uint8_t addr, uint8_t value[3]);
uint8_t nrf24_tx_flush(void);
void nrf24_delay(void);
 int nrf24_init(void);
void nrf24_set_rx_addr(uint8_t addr[3]);
void nrf24_set_tx_addr(uint8_t addr[3]);
void nrf24_rx_mode(void);
void nrf24_idle_mode(uint8_t standby);
uint8_t nrf24_rx_fifo_data(void);
uint8_t nrf24_rx_data_avail(void);
void nrf24_rx_read(uint8_t *buf, uint8_t *pkt_len);
void nrf24_tx(uint8_t *buf, uint8_t len);
int nrf24_tx_result_wait(void);
