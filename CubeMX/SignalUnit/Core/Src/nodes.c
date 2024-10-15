#include "nodes.h"

volatile uint16_t axleCounter = 0;
volatile SignalState *currentSignalState;
volatile TrainDirection trainDir = TRAIN_DIR_NOT_KNOWN;

Nodes thisNode, prevNode, nextNode;
SignalState red, doubleYellow, yellow, green;

/**
 * @brief calculates 1's complement checksum
 *
 * @param p pointer to buffer
 * @param len no. of bytes
 *
 * @return uint8_t returns the 1's complement checksum
 */
static uint8_t getChecksum(uint8_t p[], uint8_t len)
{
	unsigned short sum = 0;

	for (uint8_t i = 0; i < len; ++i)
	{
		sum += p[i];
		sum += (sum >> 8);	// one`s complement addition
	}
	sum = ~sum;
	return (uint8_t) sum;
}

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

	thisNode.nodeID = THIS_NODE_ID;
	thisNode.axleCount = axleCounter;
	thisNode.signal = SIGNAL_NOT_KNOWN;
	thisNode.nodeReady = false;

	if (THIS_NODE_NUM == TOTAL_NO_OF_NODES)
	{

		thisNode.next = NULL;
		thisNode.prev = &prevNode;
		prevNode.signal = SIGNAL_NOT_KNOWN;
		prevNode.nodeID = PREV_NODE_ID;
		prevNode.axleCount = 0;
		nextNode.nodeReady = false;
		prevNode.next = NULL;
		prevNode.prev = NULL;
		memset(prevNode.signalData, SIGNAL_NOT_KNOWN, sizeof(prevNode.signalData));

	} else if (THIS_NODE_NUM == 1)
	{
		thisNode.next = &nextNode;
		nextNode.signal = SIGNAL_NOT_KNOWN;
		nextNode.nodeID = NEXT_NODE_ID;
		nextNode.axleCount = 0;
		nextNode.nodeReady = false;
		nextNode.next = NULL;
		nextNode.prev = NULL;
		memset(nextNode.signalData, SIGNAL_NOT_KNOWN, sizeof(nextNode.signalData));

	} else
	{
		thisNode.next = &nextNode;
		thisNode.prev = &prevNode;

		nextNode.nodeID = NEXT_NODE_ID;
		nextNode.signal = SIGNAL_NOT_KNOWN;
		nextNode.axleCount = 0;
		nextNode.nodeReady = false;
		nextNode.next = NULL;
		nextNode.prev = NULL;
		memset(nextNode.signalData, SIGNAL_NOT_KNOWN, sizeof(nextNode.signalData));

		prevNode.signal = SIGNAL_NOT_KNOWN;
		prevNode.nodeID = PREV_NODE_ID;
		prevNode.axleCount = 0;
		nextNode.nodeReady = false;
		prevNode.next = NULL;
		prevNode.prev = NULL;
		memset(prevNode.signalData, SIGNAL_NOT_KNOWN, sizeof(prevNode.signalData));
	}
}

/**
 * @brief checks if the node is ready
 *
 * @param None
 *
 * @return bool node ready state
 * 		   - true node is ready
 * 		   - false node is not ready
 */
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
		memcpy(&signals[noOfNodesBefore], thisNode.next->signalData, noOfNodesAfter);
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
		thisNode.signalData[2] = (temp >> 4);	// signal state of the second node after next node

	} else if (thisNode.prev != NULL && communicatingNodeID == thisNode.prev->nodeID)
	{
		thisNode.prev->axleCount = p->receivePayload[AXLE_COUNT_INDEX];

		temp = p->receivePayload[STATUS_P3_NODE_INDEX];
		thisNode.prev->nodeReady = __GET_NODE_READY_BIT_STATE(temp);
		thisNode.prev->signalReset = __GET_SIGNAL_RESET_BIT_STATE(temp);

		temp = p->receivePayload[C_N1_NODE_INDEX];
		thisNode.signalData[0] = (temp >> 4);	// signal state of the previous node

		temp = p->receivePayload[P2_P1_NODE_INDEX];
		thisNode.signalData[1] = (temp & 0x0F);	// signal state of the first node after the previous node
		thisNode.signalData[2] = (temp >> 4);	// signal state of the second node after previous node
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
		p->transmitPayload[STATUS_P3_NODE_INDEX] &= ~(1 << __NODE_READY_BIT_INDEX);
	}

	// 4th byte
	// set the signal reset bit only if it is the first or the last node
	if (thisNode.signalReset && (THIS_NODE_NUM == 1 || THIS_NODE_NUM == TOTAL_NO_OF_NODES))
	{
		p->transmitPayload[STATUS_P3_NODE_INDEX] |= (1 << __SIGNAL_RESET_BIT_INDEX);
	}

	if (thisNode.prev != NULL)
	{
		// 4th byte
		p->transmitPayload[STATUS_P3_NODE_INDEX] |= (0x0F & thisNode.prev->signalData[2]); // signal data of the third node before the current node
		// 5th byte
		p->transmitPayload[P2_P1_NODE_INDEX] = (thisNode.prev->signalData[1] << 4) | thisNode.prev->signalData[0]; // signal data of the second and first node before the current node
	}
	// 6th byte
	p->transmitPayload[C_N1_NODE_INDEX] = ((uint8_t) thisNode.signal) << 4;	// state of current signal

	if (thisNode.next != NULL)
	{
		// 6th byte
		p->transmitPayload[C_N1_NODE_INDEX] |= (0x0F & thisNode.signalData[0]); // signal data of the first node after the current node
		// 7th byte
		p->transmitPayload[N2_N3_NODE_INDEX] = (thisNode.signalData[1] << 4) | thisNode.signalData[2]; // signal data of the second and third node after the current node
	}

	//8th byte
	p->transmitPayload[CHECKSUM_INDEX] = getChecksum(p->transmitPayload, 7);
}

/**
 * @brief updates the signal state
 *
 * @param None
 *
 * @return None
 */
void updateSignalState(void)
{
	SignalState temp = *currentSignalState;
	thisNode.axleCount = axleCounter;

	switch (trainDir)
	{
	case TO_HIGHER_NODE:

		switch (temp.state)
		{
		case RED:
			if (thisNode.axleCount == thisNode.next->axleCount)
			{
				// if train is moving towards higher node and the axle counter from the next node is equal to this node
				// than the train has passed this signal and the signal after
				if (temp.next != NULL)
					currentSignalState = temp.next;
				axleCounter = 0;
				thisNode.axleCount = 0;
			}
			break;

		case YELLOW:
			if (thisNode.next->signal == YELLOW)
			{
				// if next node is yellow than this node should now be double yellow
				if (temp.next != NULL)
					currentSignalState = temp.next;
			}

			break;

		case DOUBLE_YELLOW:
			if (thisNode.next->signal == DOUBLE_YELLOW)
			{
				// if the next node is double yellow then this node should now be green
				if (temp.next != NULL)
				{
					currentSignalState = temp.next;
				}
			}
			break;

		case GREEN:
			trainDir = TRAIN_DIR_NOT_KNOWN;
			break;

		default:
			break;
		}

		break;

	case TO_LOWER_NODE:
		switch (temp.state)
		{
		case RED:
			if (thisNode.axleCount == thisNode.prev->axleCount)
			{
				// if train is moving towards lower node and the axle counter from the previous node is equal to this node
				// than the train has passed this signal and the signal before
				if (temp.next != NULL)
					currentSignalState = temp.next;
				axleCounter = 0;
				thisNode.axleCount = 0;
			}
			break;

		case YELLOW:
			if (thisNode.prev->signal == YELLOW)
			{
				if (temp.next != NULL)
					currentSignalState = temp.next;
			}

			break;

		case DOUBLE_YELLOW:
			if (thisNode.prev->signal == DOUBLE_YELLOW)
			{
				if (temp.next != NULL)
					currentSignalState = temp.next;
			}
			break;

		case GREEN:
			trainDir = TRAIN_DIR_NOT_KNOWN;
			break;

		default:
			break;
		}

		break;

	default:
		break;
	}
}

/**
 * @brief Sets the signal lights as per the signal state
 *
 * @param None
 *
 * @return None
 */
void setSignalLeds(void)
{
	SignalState temp = *currentSignalState;
	switch (temp.state)
	{
	case RED:
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port,
		LED_GREEN_Pin | LED_YELLOW1_Pin | LED_YELLOW2_Pin, GPIO_PIN_RESET);
		break;

	case YELLOW:
		HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port, LED_YELLOW1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port,
		LED_GREEN_Pin | LED_YELLOW2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;

	case DOUBLE_YELLOW:
		HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port,
		LED_YELLOW1_Pin | LED_YELLOW2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;

	case GREEN:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port,
		LED_YELLOW1_Pin | LED_YELLOW2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}
