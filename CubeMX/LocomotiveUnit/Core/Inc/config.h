#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "main.h"
#include "nrf24.h"

#define NO_OF_NODES_TO_MONITOR 3

extern const uint8_t TOTAL_NO_OF_NODES;
extern const uint8_t NODE_IDS[], NODE_ADDRESS[][5];
extern const uint8_t LOCOMOTIVE_NODE_ID;

extern const uint8_t NRF24_TX_PWR, NRF24_DATA_RATE, NRF24_AUTO_RETRY_DELAY, NRF24_AUTO_RETRY_COUNT, RF_CHANNEL;
extern const unsigned long TX_INTERVAL, SW_SAMPLING_INTERVAL;
extern const uint16_t MAX_PWM_VAL;
#endif /* INC_CONFIG_H_ */
