#include "nodes.h"

typedef enum {
	SIGNAL_NOT_KNOWN = 0, RED, YELLOW, DOUBLE_YELLOW, GREEN
} Signal;

typedef struct signalstate {
	Signal state;
	struct signalstate *next;
} SignalState;

typedef enum {
	TRAIN_DIR_NOT_KNOWN = 0, TO_LOWER_NODE, TO_HIGHER_NODE
} TrainDirection;

typedef struct node {
	struct node *prev;
	struct node *next;
	Signal signal;
	uint8_t nodeID;
	uint8_t nodeReady;
	uint8_t axleCount;
	uint8_t signalData[3];
} Nodes;

typedef struct {
	uint8_t receivePayload[PAYLOAD_LENGTH];
	uint8_t transmitPayload[PAYLOAD_LENGTH];
} Payload;

volatile uint8_t axleCounter = 0;

Nodes thisNode, prevNode, nextNode;
SignalState red, doubleYellow, yellow, green;
volatile SignalState *currentSignalState;
volatile TrainDirection trainDir = TRAIN_DIR_NOT_KNOWN;

void signalStateInit(void) {

	axleCounter = 0;

	red.next = &yellow;
	red.state = RED;

	doubleYellow.next = &green;
	doubleYellow.state = DOUBLE_YELLOW;

	yellow.next = &doubleYellow;
	yellow.state = YELLOW;

	green.next = NULL;
	green.state = GREEN;

	currentSignalState = &green;

	thisNode.nodeID = THIS_NODE_NUM;
	thisNode.axleCount = axleCounter;
	thisNode.signal = SIGNAL_NOT_KNOWN;
	thisNode.nodeReady = false;

	if (THIS_NODE_NUM == TOTAL_NO_OF_NODES) {

		thisNode.next = NULL;
		thisNode.prev = &prevNode;
		prevNode.signal = SIGNAL_NOT_KNOWN;
		prevNode.nodeID = THIS_NODE_NUM - 1;
		prevNode.axleCount = 0;
		nextNode.nodeReady = false;
		prevNode.next = NULL;
		prevNode.prev = NULL;
		memset(prevNode.signalData, SIGNAL_NOT_KNOWN,
				sizeof(prevNode.signalData));

	} else if (THIS_NODE_NUM == 1) {
		thisNode.next = &nextNode;
		nextNode.signal = SIGNAL_NOT_KNOWN;
		nextNode.nodeID = THIS_NODE_NUM + 1;
		nextNode.axleCount = 0;
		nextNode.nodeReady = false;
		nextNode.next = NULL;
		nextNode.prev = NULL;
		memset(nextNode.signalData, SIGNAL_NOT_KNOWN,
				sizeof(nextNode.signalData));

	} else {
		thisNode.next = &nextNode;
		thisNode.prev = &prevNode;

		nextNode.nodeID = (THIS_NODE_NUM + 1);
		nextNode.signal = SIGNAL_NOT_KNOWN;
		nextNode.axleCount = 0;
		nextNode.nodeReady = false;
		nextNode.next = NULL;
		nextNode.prev = NULL;
		memset(nextNode.signalData, SIGNAL_NOT_KNOWN,
				sizeof(nextNode.signalData));

		prevNode.signal = SIGNAL_NOT_KNOWN;
		prevNode.nodeID = (THIS_NODE_NUM - 1);
		prevNode.axleCount = 0;
		nextNode.nodeReady = false;
		prevNode.next = NULL;
		prevNode.prev = NULL;
		memset(prevNode.signalData, SIGNAL_NOT_KNOWN,
				sizeof(prevNode.signalData));
	}
}

bool isNodeReady(void) {
	uint8_t signals[2 * NO_OF_NODES_TO_MONITOR];
	uint8_t noOfNodesBefore = 0, noOfNodesAfter = 0;

	// finding number of nodes before this node
	if (THIS_NODE_NUM > NO_OF_NODES_TO_MONITOR) {
		noOfNodesBefore = NO_OF_NODES_TO_MONITOR;
	} else {
		noOfNodesBefore = THIS_NODE_NUM - 1;
	}

	// finding number of nodes after this node
	if (TOTAL_NO_OF_NODES - THIS_NODE_NUM > NO_OF_NODES_TO_MONITOR) {
		noOfNodesAfter = NO_OF_NODES_TO_MONITOR;
	} else {
		noOfNodesAfter = TOTAL_NO_OF_NODES - THIS_NODE_NUM;
	}

	memset(signals, SIGNAL_NOT_KNOWN, sizeof(signals));

	if (thisNode.prev != NULL) {
		memcpy(signals, thisNode.prev->signalData, noOfNodesBefore);
	}
	if (thisNode.next != NULL) {
		memcpy(&signals[noOfNodesBefore], thisNode.next->signalData,
				noOfNodesAfter);
	}

	// if anyone of the signals is not known then the node is not ready
	for (uint8_t i = 0; i < (noOfNodesBefore + noOfNodesAfter); i++) {
		if (signals[i] == SIGNAL_NOT_KNOWN)
			return false;
	}

	return true;
}
/*
 * @brief Checks if payload is valid
 *
 * @param p pointer to Payload object
 * @param communicatingNodeID ID of the node whose payload to verify
 *
 * @return bool payload valid
 * 		   - true payload is valid
 * 		   - false payload is invalid
 */
bool isPayLoadValid(Payload *p, uint8_t communicatingNodeID) {
	// validate if payload received from current node
	if (p->receivePayload[SOURCE_ID_INDEX] != communicatingNodeID
			&& p->receivePayload[DESTINATION_ID_INDEX] != thisNode.nodeID)
		return false;

	uint8_t checksum = p->receivePayload[CHECKSUM_INDEX];
	unsigned short sum = 0;

	// validate checksum
	for (uint8_t i = SOURCE_ID_INDEX; i < CHECKSUM_INDEX; i++) {
		sum += p->receivePayload[i];
		sum += (sum >> 8);	// one`s complement addition
	}

	// for a correct payload, sum + the received checksum should be equal to 255
	if ((sum + p->receivePayload[CHECKSUM_INDEX]) & 0xFF != 0xFF)
		return false;

	return true;
}
