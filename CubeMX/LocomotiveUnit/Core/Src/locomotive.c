#include "locomotive.h"


/*
 * @brief Checks if payload is valid
 *
 * @param loco pointer to Locomotive object
 * @param p pointer to Payload object
 *
 * @return bool payload valid
 * 		   - true payload is valid
 * 		   - false payload is invalid
 */
bool isPayLoadValid(Locomotive *loco, Payload *p)
{
	// validate if payload received from current node
	if (p->receivePayload[SOURCE_ID_INDEX] != loco->id
			&& p->receivePayload[DESTINATION_ID_INDEX] != loco->comNodeID)
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
 * @param loco pointer to Locomotive object
 * @param p pointer to Payload object
 *
 * @return None
 */
void extractPayloadData(Locomotive *loco, Payload *p)
{
	uint8_t temp;

	temp = p->receivePayload[STATUS_P3_NODE_INDEX];
	loco->isComNodeReady = __GET_NODE_READY_BIT_STATE(temp);

	switch (loco->dir) {
		case TO_HIGHER_NODE:

			temp = p->receivePayload[C_N1_NODE_INDEX];
			loco->signalData[1] = (Signal)(temp & 0x0F);// signal state of the first node after the next node
			loco->signalData[0] = (Signal)(temp >> 4);	// signal state of the next node, i.e, the node that transmitted the data

			temp = p->receivePayload[N2_N3_NODE_INDEX];
			loco->signalData[3] = (Signal)(temp & 0x0F);// signal state of the third node after the next node
			loco->signalData[2] = (Signal)(temp >> 4);	// signal state of the second node after next node
			break;

		case TO_LOWER_NODE:
			temp = p->receivePayload[C_N1_NODE_INDEX];
			loco->signalData[0] = (Signal)(temp >> 4);	// signal state of the previous node, i.e, the node that transmitted the data

			temp = p->receivePayload[P2_P1_NODE_INDEX];
			loco->signalData[1] = (Signal)(temp & 0x0F);	// signal state of the first node after the previous node
			loco->signalData[2] = (Signal)(temp >> 4);	// signal state of the second node after previous node

			temp = p->receivePayload[STATUS_P3_NODE_INDEX];

			loco->signalData[3] = (Signal)(temp & 0x0F);// signal state of the third node after previous node
			break;

		default:
			break;

	}
}

/**
 * @brief updates locomotive and line state
 *
 * @param loco pointer to Locomotive object
 *
 * @return None
 */
void updateLocomotiveState(Locomotive *loco)
{

	switch (loco->signalData[0])
	{
	case GREEN:
		loco->state = GO;
		loco->line = CLEAR;
		break;

	case DOUBLE_YELLOW:
		loco->state = GO;
		loco->line = BLOCK;
		break;

	case YELLOW:
		loco->state = SLOW_DOWN;
		loco->line = BLOCK;
		break;

	case RED:
		loco->state = STOP;
		loco->line = BLOCK;
		break;

	default:
		break;
	}

	// search for RED signal
	uint8_t *ptr = (uint8_t*) memchr(loco->signalData, RED, NO_OF_NODES_TO_MONITOR + 1);

	uint8_t index = 0;
	if (ptr != NULL)
	{
		// get array index at which RED signal is found
		index = ptr - loco->signalData;

		switch (index)
		{
		case 0:
			loco->state = STOP;
			loco->line = BLOCK;
			break;

		case 1:
			// train incoming
			if (loco->signalData[2] != GREEN && loco->signalData[2] != SIGNAL_NOT_KNOWN)
			{
				loco->state = STOP;
				loco->line = BLOCK;
			} else
			{
				loco->state = SLOW_DOWN;
			}
			break;

		case 2:
			// train incoming
			if (loco->signalData[3] != GREEN && loco->signalData[3] != SIGNAL_NOT_KNOWN)
			{
				loco->state = STOP;
				loco->line = BLOCK;
			} else
			{
				loco->state = SLOW_DOWN;
			}

			break;

		default:
			break;
		}
	}
}

