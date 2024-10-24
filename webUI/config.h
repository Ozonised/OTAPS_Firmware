#ifndef __CONFIG_H
#define __CONFIG_H

#define WEB_PAGE_UPDATE_INTERVAL 500
#define SIGNAL_NODE_IIC_ADDRESS 0x06

#define SIGNAL_NODE_ID 0x01
#define FIRST_SIGNAL_NODE_ID 0x01
#define LAST_SIGNAL_NODE_ID 0x05

#define IIC_TRANSMIT_INTERVAL 200
#define IIC_SIGNAL_RESET_INDEX 0
#define IIC_HOME_SIGNAL_STATE_INDEX 1

#define IIC_SIGNAL_NODE_PAYLOAD_LENGTH 8
#define IIC_SOURCE_ID_INDEX 0
#define IIC_DESTINATION_ID_INDEX 1
#define IIC_AXLE_COUNT_INDEX 2
#define IIC_STATUS_P3_NODE_INDEX 3	// index for status flag and 3rd node before the current node
#define IIC_P2_P1_NODE_INDEX 4		// index for 2nd and 1st node before the current node
#define IIC_C_N1_NODE_INDEX 5		// index for current node and next node after current node
#define IIC_N2_N3_NODE_INDEX 6		// index for 2nd and 3rd node after the current node
#define IIC_CHECKSUM_INDEX 7

#endif