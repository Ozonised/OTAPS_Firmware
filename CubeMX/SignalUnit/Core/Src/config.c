/*
 * config.c
 *
 *  Created on: Oct 6, 2024
 *      Author: farhan
 */

#include "config.h"

const uint8_t THIS_NODE_NUM = 1;
const uint8_t TOTAL_NO_OF_NODES = 5;

// Node addresses
const uint8_t THIS_NODE_ADDRESS[5] = "NRF14";
const uint8_t PREV_NODE_ADDRESS[5] = "NRF14";
const uint8_t NEXT_NODE_ADDRESS[5] = "NRF14";
const uint8_t LOCOMOTIVE_NODE_ADDRESS[5] = "LOCOT";

// Node ID
// for the sake of simplicity, node number will the node ID
const uint8_t THIS_NODE_ID = THIS_NODE_NUM;
const uint8_t PREV_NODE_ID = THIS_NODE_NUM - 1;
const uint8_t NEXT_NODE_ID = THIS_NODE_NUM + 1;

// RF Channel
const uint8_t NRF24_LOWER_NODE_RF_CHANNEL = 1;
const uint8_t NRF24_HIGHER_NODE_RF_CHANNEL = 2;
const uint8_t NRF24_LOCOMOTIVE_NODE_RF_CHANNEL = 107;
const uint8_t NRF24_RF_CHANNEL_MIN = 1, NRF24_RF_CHANNEL_MAX = 105;

// NRF24 module config
const uint8_t NRF24_TX_PWR = nRF24_TXPWR_18dBm;	// one of nRF24_TXPWR_xx values
const uint8_t NRF24_DATA_RATE = nRF24_DR_250kbps;	// one of nRF24_DR_xx values
const uint8_t NRF24_AUTO_RETRY_DELAY = nRF24_ARD_1250us;	// one of nRF24_ARD_xx values
const uint8_t NRF24_AUTO_RETRY_COUNT = 10;	// from 0 - 15

const unsigned long TX_INTERVAL = 166, SW_SAMPLING_INTERVAL = 15;

