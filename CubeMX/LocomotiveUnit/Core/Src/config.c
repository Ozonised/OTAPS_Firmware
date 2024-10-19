#include "config.h"

const uint8_t TOTAL_NO_OF_NODES = 5;
const uint8_t NODE_IDS[] = {1, 2, 3, 4, 5};
const uint8_t NODE_ADDRESS[5] = {};

// NRF24 module config
const uint8_t NRF24_TX_PWR = nRF24_TXPWR_18dBm;	// one of nRF24_TXPWR_xx values
const uint8_t NRF24_DATA_RATE = nRF24_DR_250kbps;	// one of nRF24_DR_xx values
const uint8_t NRF24_AUTO_RETRY_DELAY = nRF24_ARD_2250us;	// one of nRF24_ARD_xx values
const uint8_t NRF24_AUTO_RETRY_COUNT = 10;	// from 0 - 15

const unsigned long TX_INTERVAL = 200, SW_SAMPLING_INTERVAL = 15;
