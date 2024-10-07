/*
 * config.h
 *
 *  Created on: Oct 6, 2024
 *      Author: farhan
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

#define NO_OF_NODES_TO_MONITOR 3

extern const uint8_t THIS_NODE_NUM, TOTAL_NO_OF_NODES;
extern const uint8_t THIS_NODE_ADDRESS[5], PREV_NODE_ADDRESS[5], LOCOMOTIVE_NODE_ADDRESS[5];

#endif /* INC_CONFIG_H_ */
