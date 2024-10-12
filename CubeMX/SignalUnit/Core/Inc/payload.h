/*
 * payload.h
 *
 *  Created on: Oct 6, 2024
 *      Author: farhan
 */

#ifndef INC_PAYLOAD_H_
#define INC_PAYLOAD_H_

#define PAYLOAD_LENGTH 8

#define SOURCE_ID_INDEX 0
#define DESTINATION_ID_INDEX 1
#define AXLE_COUNT_INDEX 2
#define STATUS_P3_NODE_INDEX 3	// index for status flag and 3rd node before the current node
#define P2_P1_NODE_INDEX 4		// index for 2nd and 1st node before the current node
#define C_N1_NODE_INDEX 5		// index for current node and next node after current node
#define N2_N3_NODE_INDEX 6		// index for 2nd and 3rd node after the current node
#define CHECKSUM_INDEX 7

#define __NODE_READY_BIT_INDEX 4
#define __SIGNAL_RESET_BIT_INDEX 5
#define __GET_SIGNAL_RESET_BIT_STATE(BYTE) ((BYTE & (1 << __SIGNAL_RESET_BIT_INDEX)) >> __SIGNAL_RESET_BIT_INDEX)
#define __GET_NODE_READY_BIT_STATE(BYTE) ((BYTE & (1 << __NODE_READY_BIT_INDEX)) >> __NODE_READY_BIT_INDEX)
#endif /* INC_PAYLOAD_H_ */
