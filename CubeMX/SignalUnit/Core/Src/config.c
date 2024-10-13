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

