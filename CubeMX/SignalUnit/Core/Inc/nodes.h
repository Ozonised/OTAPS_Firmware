/*
 * nodes.h
 *
 *  Created on: Oct 6, 2024
 *      Author: farhan
 */

#ifndef INC_NODES_H_
#define INC_NODES_H_

#include "main.h"
#include "nrf24.h"
#include "config.h"
#include "payload.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
	SIGNAL_NOT_KNOWN = 0, RED, YELLOW, DOUBLE_YELLOW, GREEN
} Signal;

typedef struct signalstate
{
	Signal state;
	struct signalstate *next;
} SignalState;

typedef enum
{
	TRAIN_DIR_NOT_KNOWN = 0, TO_LOWER_NODE, TO_HIGHER_NODE
} TrainDirection;

typedef struct node
{
	struct node *prev;
	struct node *next;
	Signal signal;
	uint8_t nodeID;
	uint8_t nodeReady;
	uint8_t signalReset;
	uint8_t axleCount;
	uint8_t signalData[3];
} Nodes;

typedef struct
{
	uint8_t receivePayload[PAYLOAD_LENGTH];
	uint8_t transmitPayload[PAYLOAD_LENGTH];
} Payload;

extern Nodes thisNode;
extern SignalState red, doubleYellow, yellow, green;

void signalStateInit(void);
bool isNodeReady(void);
bool isPayLoadValid(Payload *p, uint8_t communicatingNodeID);
void extractPayloadData(Payload *p, uint8_t communicatingNodeID);
void updateTxPayload(Payload *p, uint8_t communicatingNodeID);
void updateSignalState(void);
void setSignalLeds(void);

#endif /* INC_NODES_H_ */
