#ifndef INC_LOCOMOTIVE_H_
#define INC_LOCOMOTIVE_H_

#include "main.h"
#include "nrf24.h"
#include "config.h"
#include "stdbool.h"

typedef enum
{
    TRAIN_DIR_NOT_KNOWN,
    TO_LOWER_NODE,
    TO_HIGHER_NODE
} TrainDirection;

typedef enum
{
    SIGNAL_NOT_KNOWN,
    RED,
    YELLOW,
    DOUBLE_YELLOW,
    GREEN
} Signal;

typedef enum
{
    LINE_STATE_NOT_KNOWN,
    CLEAR,
    BLOCK
} LineState;

typedef enum
{
    STOP,
    SLOW_DOWN,
    GO
} TrainState;

typedef struct
{
    TrainDirection dir;
    TrainState state;
    LineState line;
    Signal signalData[NO_OF_NODES_TO_MONITOR + 1];
    uint8_t comNodeNo;
    bool isComNodeReady;
} Locomotive;

extern Locomotive locomotive;

#endif /* INC_LOCOMOTIVE_H_ */
