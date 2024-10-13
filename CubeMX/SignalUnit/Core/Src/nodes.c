#include "nodes.h"

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

static volatile uint16_t axleCounter = 0;

Nodes thisNode, prevNode, nextNode;
SignalState red, doubleYellow, yellow, green;
volatile SignalState *currentSignalState;
volatile TrainDirection trainDir = TRAIN_DIR_NOT_KNOWN;

void signalStateInit(void)
{

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

	if (THIS_NODE_NUM == TOTAL_NO_OF_NODES)
	{

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

	} else if (THIS_NODE_NUM == 1)
	{
		thisNode.next = &nextNode;
		nextNode.signal = SIGNAL_NOT_KNOWN;
		nextNode.nodeID = THIS_NODE_NUM + 1;
		nextNode.axleCount = 0;
		nextNode.nodeReady = false;
		nextNode.next = NULL;
		nextNode.prev = NULL;
		memset(nextNode.signalData, SIGNAL_NOT_KNOWN,
				sizeof(nextNode.signalData));

	} else
	{
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

bool isNodeReady(void)
{
	uint8_t signals[2 * NO_OF_NODES_TO_MONITOR];
	uint8_t noOfNodesBefore = 0, noOfNodesAfter = 0;

	// finding number of nodes before this node
	if (THIS_NODE_NUM > NO_OF_NODES_TO_MONITOR)
	{
		noOfNodesBefore = NO_OF_NODES_TO_MONITOR;
	} else
	{
		noOfNodesBefore = THIS_NODE_NUM - 1;
	}

	// finding number of nodes after this node
	if (TOTAL_NO_OF_NODES - THIS_NODE_NUM > NO_OF_NODES_TO_MONITOR)
	{
		noOfNodesAfter = NO_OF_NODES_TO_MONITOR;
	} else
	{
		noOfNodesAfter = TOTAL_NO_OF_NODES - THIS_NODE_NUM;
	}

	memset(signals, SIGNAL_NOT_KNOWN, sizeof(signals));

	if (thisNode.prev != NULL)
	{
		memcpy(signals, thisNode.prev->signalData, noOfNodesBefore);
	}
	if (thisNode.next != NULL)
	{
		memcpy(&signals[noOfNodesBefore], thisNode.next->signalData,
				noOfNodesAfter);
	}

	// if anyone of the signals is not known then the node is not ready
	for (uint8_t i = 0; i < (noOfNodesBefore + noOfNodesAfter); i++)
	{
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
bool isPayLoadValid(Payload *p, uint8_t communicatingNodeID)
{
	// validate if payload received from current node
	if (p->receivePayload[SOURCE_ID_INDEX] != communicatingNodeID
			&& p->receivePayload[DESTINATION_ID_INDEX] != thisNode.nodeID)
		return false;

	unsigned short sum = 0;

	// validate checksum
	for (uint8_t i = SOURCE_ID_INDEX; i < CHECKSUM_INDEX; i++)
	{
		sum += p->receivePayload[i];
		sum += (sum >> 8);	// one`s complement addition
	}

	// for a correct payload, sum + the received checksum should be equal to 255
	if (((sum + p->receivePayload[CHECKSUM_INDEX]) & 0xFF) != 0xFF)
		return false;

	return true;
}

/**
 * @brief Extracts the payload data into the respective Node object
 *
 * @param p pointer to Payload object
 * @param communicatingNodeID ID of the node whose payload to extract
 *
 * @return None
 */
void extractPayloadData(Payload *p, uint8_t communicatingNodeID)
{
	uint8_t temp;

	if (thisNode.next != NULL && communicatingNodeID == thisNode.next->nodeID)
	{
		thisNode.next->axleCount = p->receivePayload[AXLE_COUNT_INDEX];

		temp = p->receivePayload[STATUS_P3_NODE_INDEX];
		thisNode.next->nodeReady = __GET_NODE_READY_BIT_STATE(temp);
		thisNode.next->signalReset = __GET_SIGNAL_RESET_BIT_STATE(temp);

		temp = p->receivePayload[C_N1_NODE_INDEX];
		thisNode.signalData[1] = (temp & 0x0F);	// signal state of the first node after the next node
		thisNode.signalData[0] = (temp >> 4);	// signal state of the next node

		temp = p->receivePayload[N2_N3_NODE_INDEX];
		thisNode.signalData[2] = (temp >> 4);// signal state of the second node after next node

	} else if (thisNode.prev != NULL
			&& communicatingNodeID == thisNode.prev->nodeID)
	{
		thisNode.prev->axleCount = p->receivePayload[AXLE_COUNT_INDEX];

		temp = p->receivePayload[STATUS_P3_NODE_INDEX];
		thisNode.prev->nodeReady = __GET_NODE_READY_BIT_STATE(temp);
		thisNode.prev->signalReset = __GET_SIGNAL_RESET_BIT_STATE(temp);

		temp = p->receivePayload[C_N1_NODE_INDEX];
		thisNode.signalData[0] = (temp >> 4);// signal state of the previous node

		temp = p->receivePayload[P2_P1_NODE_INDEX];
		thisNode.signalData[1] = (temp & 0x0F);	// signal state of the first node after the previous node
		thisNode.signalData[2] = (temp >> 4);// signal state of the second node after previous node
	}
}

/**
 * @brief updates the transmit payload
 *
 * @param p pointer to payload object
 * @param communicatingNodeID ID of the node to send data to
 *
 * @return None
 */
void updateTxPayload(Payload *p, uint8_t communicatingNodeID)
{
	memset(p->transmitPayload, 0, PAYLOAD_LENGTH);

	// 1st byte
	p->transmitPayload[SOURCE_ID_INDEX] = thisNode.nodeID;	// Source ID

	// 2nd byte
	// destination ID
	if (communicatingNodeID == thisNode.prev->nodeID)
	{
		p->transmitPayload[DESTINATION_ID_INDEX] = thisNode.prev->nodeID;
	} else if (communicatingNodeID == thisNode.next->nodeID)
	{
		p->transmitPayload[DESTINATION_ID_INDEX] = thisNode.next->nodeID;
	}

	// 3rd byte
	p->transmitPayload[AXLE_COUNT_INDEX] = thisNode.axleCount; // axle count

	// 4th byte
	// set node ready bit
	if (thisNode.nodeReady)
	{
		p->transmitPayload[STATUS_P3_NODE_INDEX] |= (1 << __NODE_READY_BIT_INDEX);
	} else
	{
		p->transmitPayload[STATUS_P3_NODE_INDEX] &= ~(1
				<< __NODE_READY_BIT_INDEX);
	}

	// 4th byte
	// set the signal reset bit only if it is the first or the last node
	if (thisNode.signalReset
			&& (THIS_NODE_NUM == 1 || THIS_NODE_NUM == TOTAL_NO_OF_NODES))
	{
		p->transmitPayload[STATUS_P3_NODE_INDEX] |= (1
				<< __SIGNAL_RESET_BIT_INDEX);
	}

	if (thisNode.prev != NULL)
	{
		// 4th byte
		p->transmitPayload[STATUS_P3_NODE_INDEX] |= (0x0F
				& thisNode.prev->signalData[2]); // signal data of the third node before the current node
		// 5th byte
		p->transmitPayload[P2_P1_NODE_INDEX] = (thisNode.prev->signalData[1]
				<< 4) | thisNode.prev->signalData[0]; // signal data of the second and first node before the current node
	}
	// 6th byte
	p->transmitPayload[C_N1_NODE_INDEX] = ((uint8_t) thisNode.signal) << 4;	// state of current signal

	if (thisNode.next != NULL)
	{
		// 6th byte
		p->transmitPayload[C_N1_NODE_INDEX] |= (0x0F & thisNode.signalData[0]); // signal data of the first node after the current node
		// 7th byte
		p->transmitPayload[N2_N3_NODE_INDEX] = (thisNode.signalData[1] << 4)
				| thisNode.signalData[2]; // signal data of the second and third node after the current node
	}
}
