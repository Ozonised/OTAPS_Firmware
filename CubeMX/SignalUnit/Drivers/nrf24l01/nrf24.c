// Functions to manage the nRF24L01+ transceiver

#include "nrf24.h"

// Read a register
// input:
//   nrf24 - pointer to NRF24 object
//   reg - number of register to read
// return: value of register
static uint8_t nRF24_ReadReg(NRF24 *nrf24, uint8_t reg)
{
	uint8_t value;

	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nrf24, nRF24_CMD_NOP);
	nRF24_CSN_H(nrf24);

	return value;
}

// Write a new value to register
// input:
//   nrf24 - pointer to NRF24 object
//   reg - number of register to write
//   value - value to write
static void nRF24_WriteReg(NRF24 *nrf24, uint8_t reg, uint8_t value)
{
	nRF24_CSN_L(nrf24);
	if (reg < nRF24_CMD_W_REGISTER) {
		// This is a register access
		nRF24_LL_RW(nrf24, nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP ));
		nRF24_LL_RW(nrf24, value);
	} else {
		// This is a single byte command or future command/register
		nRF24_LL_RW(nrf24, reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
				(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
			// Send register value
			nRF24_LL_RW(nrf24, value);
		}
	}
	nRF24_CSN_H(nrf24);
}

// Read a multi-byte register
// input:
//   nrf24 - pointer to NRF24 object
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
static void nRF24_ReadMBReg(NRF24 *nrf24, uint8_t reg, uint8_t *pBuf,
		uint8_t count)
{
	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, reg);
	while (count--) {
		*pBuf++ = nRF24_LL_RW(nrf24, nRF24_CMD_NOP);
	}
	nRF24_CSN_H(nrf24);
}

// Write a multi-byte register
// input:
//   nrf24 - pointer to NRF24 object
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
static void nRF24_WriteMBReg(NRF24 *nrf24, uint8_t reg, uint8_t *pBuf,
		uint8_t count)
{
	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, reg);
	while (count--) {
		nRF24_LL_RW(nrf24, *pBuf++);
	}
	nRF24_CSN_H(nrf24);
}

// Set transceiver to it's initial state
// input:
//   nrf24 - pointer to NRF24 object
// note: RX/TX pipe addresses remains untouched
void nRF24_Init(NRF24 *nrf24)
{
	// Write to registers their initial values
	nRF24_WriteReg(nrf24, nRF24_REG_CONFIG, 0x08);
	nRF24_WriteReg(nrf24, nRF24_REG_EN_AA, 0x3F);
	nRF24_WriteReg(nrf24, nRF24_REG_EN_RXADDR, 0x03);
	nRF24_WriteReg(nrf24, nRF24_REG_SETUP_AW, 0x03);
	nRF24_WriteReg(nrf24, nRF24_REG_SETUP_RETR, 0x03);
	nRF24_WriteReg(nrf24, nRF24_REG_RF_CH, 0x02);
	nRF24_WriteReg(nrf24, nRF24_REG_RF_SETUP, 0x0E);
	nRF24_WriteReg(nrf24, nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nrf24, nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	nRF24_FlushRX(nrf24);
	nRF24_FlushTX(nrf24);

	// Clear any pending interrupt flags
	nRF24_ClearIRQFlags(nrf24);

	// Deassert CSN pin (chip release)
	nRF24_CSN_H(nrf24);
}

// Check if the nRF24L01 present
// input:
//   nrf24 - pointer to NRF24 object
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(NRF24 *nrf24)
{
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	nRF24_WriteMBReg(nrf24, nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nrf24, nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;
}

// Control transceiver power mode
// input:
//   nrf24 - pointer to NRF24 object
//   mode - new state of power mode, one of nRF24_PWR_xx values
void nRF24_SetPowerMode(NRF24 *nrf24, uint8_t mode)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nrf24, nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nrf24, nRF24_REG_CONFIG, reg);
}

// Set transceiver operational mode
// input:
//   nrf24 - pointer to NRF24 object
//   mode - operational mode, one of nRF24_MODE_xx values
void nRF24_SetOperationalMode(NRF24 *nrf24, uint8_t mode)
{
	uint8_t reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	nRF24_WriteReg(nrf24, nRF24_REG_CONFIG, reg);
}

// Set transceiver DynamicPayloadLength feature for all the pipes
// input:
//   nrf24 - pointer to NRF24 object
//   mode - status, one of nRF24_DPL_xx values
void nRF24_SetDynamicPayloadLength(NRF24 *nrf24, uint8_t mode)
{
	uint8_t reg;
	reg = nRF24_ReadReg(nrf24, nRF24_REG_FEATURE);
	if(mode) {
		nRF24_WriteReg(nrf24, nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL);
		nRF24_WriteReg(nrf24, nRF24_REG_DYNPD, 0x1F);
	} else {
		nRF24_WriteReg(nrf24, nRF24_REG_FEATURE, reg & ~ nRF24_FEATURE_EN_DPL);
		nRF24_WriteReg(nrf24, nRF24_REG_DYNPD, 0x0);
	}
}

// Enables Payload With Ack. NB Refer to the datasheet for proper retransmit timing.
// input:
//   nrf24 - pointer to NRF24 object
//   mode - status, 1 or 0
void nRF24_SetPayloadWithAck(NRF24 *nrf24, uint8_t mode)
{
	uint8_t reg;
	reg = nRF24_ReadReg(nrf24, nRF24_REG_FEATURE);
	if(mode) {
		nRF24_WriteReg(nrf24, nRF24_REG_FEATURE,
				reg | nRF24_FEATURE_EN_ACK_PAY);
	} else {
		nRF24_WriteReg(nrf24, nRF24_REG_FEATURE,
				reg & ~ nRF24_FEATURE_EN_ACK_PAY);
	}
}

// Configure transceiver CRC scheme
// input:
//   nrf24 - pointer to NRF24 object
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
//       enabled for at least one RX pipe
void nRF24_SetCRCScheme(NRF24 *nrf24, uint8_t scheme)
{
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	nRF24_WriteReg(nrf24, nRF24_REG_CONFIG, reg);
}

// Set frequency channel
// input:
//   nrf24 - pointer to NRF24 object
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void nRF24_SetRFChannel(NRF24 *nrf24, uint8_t channel)
{
	nRF24_WriteReg(nrf24, nRF24_REG_RF_CH, channel);
}

// Set automatic retransmission parameters
// input:
//   nrf24 - pointer to NRF24 object
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void nRF24_SetAutoRetr(NRF24 *nrf24, uint8_t ard, uint8_t arc)
{
	// Set auto retransmit settings (SETUP_RETR register)
	nRF24_WriteReg(nrf24, nRF24_REG_SETUP_RETR,
			(uint8_t) ((ard << 4) | (arc & nRF24_MASK_RETR_ARC )));
}

// Set of address widths
// input:
//   nrf24 - pointer to NRF24 object
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void nRF24_SetAddrWidth(NRF24 *nrf24, uint8_t addr_width)
{
	nRF24_WriteReg(nrf24, nRF24_REG_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   nrf24 - pointer to NRF24 object
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void nRF24_SetAddr(NRF24 *nrf24, uint8_t pipe, const uint8_t *addr)
{
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case nRF24_PIPETX:
		case nRF24_PIPE0:
		case nRF24_PIPE1:
			// Get address width
		addr_width = nRF24_ReadReg(nrf24, nRF24_REG_SETUP_AW) + 1;
			// Write address in reverse order (LSByte first)
			addr += addr_width;
		nRF24_CSN_L(nrf24);
		nRF24_LL_RW(nrf24, nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
			do {
			nRF24_LL_RW(nrf24, *addr--);
			} while (addr_width--);
		nRF24_CSN_H(nrf24);
			break;
		case nRF24_PIPE2:
		case nRF24_PIPE3:
		case nRF24_PIPE4:
		case nRF24_PIPE5:
			// Write address LSBbyte (only first byte from the addr buffer)
		nRF24_WriteReg(nrf24, nRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}

// Configure RF output power in TX mode
// input:
//   nrf24 - pointer to NRF24 object
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void nRF24_SetTXPower(NRF24 *nrf24, uint8_t tx_pwr)
{
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nrf24, nRF24_REG_RF_SETUP, reg);
}

// Configure transceiver data rate
// input:
//   nrf24 - pointer to NRF24 object
//   data_rate - data rate, one of nRF24_DR_xx values
void nRF24_SetDataRate(NRF24 *nrf24, uint8_t data_rate)
{
	uint8_t reg;

	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nrf24, nRF24_REG_RF_SETUP, reg);
}

// Configure a specified RX pipe
// input:
//   nrf24 - pointer to NRF24 object
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nRF24_SetRXPipe(NRF24 *nrf24, uint8_t pipe, uint8_t aa_state,
		uint8_t payload_len)
{
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (nRF24_ReadReg(nrf24, nRF24_REG_EN_RXADDR) | (1 << pipe))
			& nRF24_MASK_EN_RX;
	nRF24_WriteReg(nrf24, nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nRF24_WriteReg(nrf24, nRF24_RX_PW_PIPE[pipe],
			payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = nRF24_ReadReg(nrf24, nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nrf24, nRF24_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   nrf24 - pointer to NRF24 object
//   PIPE - number of RX pipe, value from 0 to 5
void nRF24_ClosePipe(NRF24 *nrf24, uint8_t pipe)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nrf24, nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nrf24, nRF24_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   nrf24 - pointer to NRF24 object
//   pipe - number of the RX pipe, value from 0 to 5
void nRF24_EnableAA(NRF24 *nrf24, uint8_t pipe)
{
	uint8_t reg;

	// Set bit in EN_AA register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nrf24, nRF24_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   nrf24 - pointer to NRF24 object
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void nRF24_DisableAA(NRF24 *nrf24, uint8_t pipe)
{
	uint8_t reg;

	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		nRF24_WriteReg(nrf24, nRF24_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg = nRF24_ReadReg(nrf24, nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nrf24, nRF24_REG_EN_AA, reg);
	}
}

// Get value of the STATUS register
// input:
//   nrf24 - pointer to NRF24 object
// return: value of STATUS register
uint8_t nRF24_GetStatus(NRF24 *nrf24)
{
	return nRF24_ReadReg(nrf24, nRF24_REG_STATUS);
}

// Get pending IRQ flags
// input:
//   nrf24 - pointer to NRF24 object
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t nRF24_GetIRQFlags(NRF24 *nrf24)
{
	return (nRF24_ReadReg(nrf24, nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ );
}

// Get status of the RX FIFO
// input:
//   nrf24 - pointer to NRF24 object
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t nRF24_GetStatus_RXFIFO(NRF24 *nrf24)
{
	return (nRF24_ReadReg(nrf24, nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO );
}

// Get status of the TX FIFO
// input:
//   nrf24 - pointer to NRF24 object
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t nRF24_GetStatus_TXFIFO(NRF24 *nrf24)
{
	return ((nRF24_ReadReg(nrf24, nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO )
			>> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// input:
//   nrf24 - pointer to NRF24 object
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(NRF24 *nrf24)
{
	return ((nRF24_ReadReg(nrf24, nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO ) >> 1);
}

// Get auto retransmit statistic
// input:
//   nrf24 - pointer to NRF24 object
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t nRF24_GetRetransmitCounters(NRF24 *nrf24)
{
	return (nRF24_ReadReg(nrf24, nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
// input:
//   nrf24 - pointer to NRF24 object
void nRF24_ResetPLOS(NRF24 *nrf24)
{
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_RF_CH);
	nRF24_WriteReg(nrf24, nRF24_REG_RF_CH, reg);
}

// Flush the TX FIFO
// input:
//   nrf24 - pointer to NRF24 object
void nRF24_FlushTX(NRF24 *nrf24)
{
	nRF24_WriteReg(nrf24, nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

// Flush the RX FIFO
// input:
//   nrf24 - pointer to NRF24 object
void nRF24_FlushRX(NRF24 *nrf24)
{
	nRF24_WriteReg(nrf24, nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

// Clear any pending IRQ flags
// input:
//   nrf24 - pointer to NRF24 object
void nRF24_ClearIRQFlags(NRF24 *nrf24)
{
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg = nRF24_ReadReg(nrf24, nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nrf24, nRF24_REG_STATUS, reg);
}

// Write TX payload
// input:
//   nrf24 - pointer to NRF24 object
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void nRF24_WritePayload(NRF24 *nrf24, uint8_t *pBuf, uint8_t length)
{
	nRF24_WriteMBReg(nrf24, nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

// Get receiver dynamic payload width
// input:
//   nrf24 - pointer to NRF24 object
// return: received payload width
static uint8_t nRF24_GetRxDplPayloadWidth(NRF24 *nrf24)
{
	uint8_t value;

	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, nRF24_CMD_R_RX_PL_WID);
	value = nRF24_LL_RW(nrf24, nRF24_CMD_NOP);
	nRF24_CSN_H(nrf24);

	return value;

}

// Reads the payload in RX buffer
// input:
//   nrf24 - pointer to NRF24 object
//	 pbuf - pointer to rx buffer
//	 length - pointer to the rx buffer length
// 	 dpl - dynamic payload mode (1 = ON & 0 = OFF)
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
static nRF24_RXResult nRF24_ReadPayloadGeneric(NRF24 *nrf24, uint8_t *pBuf,
		uint8_t *length, uint8_t dpl)
{
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (nRF24_ReadReg(nrf24, nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO ) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		if(dpl) {
			*length = nRF24_GetRxDplPayloadWidth(nrf24);
			if(*length>32) { //broken packet
				*length = 0;
				nRF24_FlushRX(nrf24);
			}
		} else {
			*length = nRF24_ReadReg(nrf24, nRF24_RX_PW_PIPE[pipe]);
		}

		// Read a payload from the RX FIFO
		if (*length) {
			nRF24_ReadMBReg(nrf24, nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}

// Read top level payload available in the RX FIFO
// input:
//   nrf24 - pointer to NRF24 object
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayload(NRF24 *nrf24, uint8_t *pBuf, uint8_t *length)
{
	return nRF24_ReadPayloadGeneric(nrf24, pBuf, length, 0);
}

// Read top level payload available in the RX FIFO with dpl enabled
// input:
//   nrf24 - pointer to NRF24 object
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayloadDpl(NRF24 *nrf24, uint8_t *pBuf,
		uint8_t *length)
{
	return nRF24_ReadPayloadGeneric(nrf24, pBuf, length, 1);
}

uint8_t nRF24_GetFeatures(NRF24 *nrf24)
{
	return nRF24_ReadReg(nrf24, nRF24_REG_FEATURE);
}
void nRF24_ActivateFeatures(NRF24 *nrf24)
{
	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, nRF24_CMD_ACTIVATE);
	nRF24_LL_RW(nrf24, 0x73);
	nRF24_CSN_H(nrf24);
}

// Writes the auto-ack payload
// input:
//   nrf24 - pointer to NRF24 object
//	 pipe - pipe number to write the data to, one of nRF24_RX_xx values
//	 payload - pointer to the buffer
//	 length - length of the payload buffer
void nRF24_WriteAckPayload(NRF24 *nrf24, nRF24_RXResult pipe, char *payload,
		uint8_t length)
{
	nRF24_CSN_L(nrf24);
	nRF24_LL_RW(nrf24, nRF24_CMD_W_ACK_PAYLOAD | pipe);
	while (length--) {
		nRF24_LL_RW(nrf24, (uint8_t) *payload++);
	}
	nRF24_CSN_H(nrf24);

}

// Function to transmit data packet
// input:
//   nrf24 - pointer to NRF24 object
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(NRF24 *nrf24, uint8_t *pBuf, uint8_t length,
		uint8_t timeout)
{
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L(nrf24);

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(nrf24, pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H(nrf24);

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do
	{
		status = nRF24_GetStatus(nrf24);
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT ))
		{
			break;
		}
		HAL_Delay(1);
	} while (timeout--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L(nrf24);

	if (!timeout)
	{
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Clear pending IRQ flags
	nRF24_ClearIRQFlags(nrf24);

	if (status & nRF24_FLAG_MAX_RT)
	{
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS)
	{
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX(nrf24);

	return nRF24_TX_ERROR;
}

/*

// Print nRF24L01+ current configuration (for debug purposes)
void nRF24_DumpConfig(void) {
	uint8_t i,j;
	uint8_t aw;
	uint8_t buf[5];

	// Dump nRF24L01+ configuration
	// CONFIG
	i = nRF24_ReadReg(nRF24_REG_CONFIG);
	USART_printf(USART1,"[0x%02X] 0x%02X MASK:%03b CRC:%02b PWR:%s MODE:P%s\r\n",
			nRF24_REG_CONFIG,
			i,
			i >> 4,
			(i & 0x0c) >> 2,
			(i & 0x02) ? "ON" : "OFF",
			(i & 0x01) ? "RX" : "TX"
		);
	// EN_AA
	i = nRF24_ReadReg(nRF24_REG_EN_AA);
	USART_printf(USART1,"[0x%02X] 0x%02X ENAA: ",nRF24_REG_EN_AA,i);
	for (j = 0; j < 6; j++) {
		USART_printf(USART1,"[P%1u%s]%s",j,
				(i & (1 << j)) ? "+" : "-",
				(j == 5) ? "\r\n" : " "
			);
	}
	// EN_RXADDR
	i = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	USART_printf(USART1,"[0x%02X] 0x%02X EN_RXADDR: ",nRF24_REG_EN_RXADDR,i);
	for (j = 0; j < 6; j++) {
		USART_printf(USART1,"[P%1u%s]%s",j,
				(i & (1 << j)) ? "+" : "-",
				(j == 5) ? "\r\n" : " "
			);
	}
	// SETUP_AW
	i = nRF24_ReadReg(nRF24_REG_SETUP_AW);
	aw = (i & 0x03) + 2;
	USART_printf(USART1,"[0x%02X] 0x%02X EN_RXADDR=%06b (address width = %u)\r\n",nRF24_REG_SETUP_AW,i,i & 0x03,aw);
	// SETUP_RETR
	i = nRF24_ReadReg(nRF24_REG_SETUP_RETR);
	USART_printf(USART1,"[0x%02X] 0x%02X ARD=%04b ARC=%04b (retr.delay=%uus, count=%u)\r\n",
			nRF24_REG_SETUP_RETR,
			i,
			i >> 4,
			i & 0x0F,
			((i >> 4) * 250) + 250,
			i & 0x0F
		);
	// RF_CH
	i = nRF24_ReadReg(nRF24_REG_RF_CH);
	USART_printf(USART1,"[0x%02X] 0x%02X (%.3uGHz)\r\n",nRF24_REG_RF_CH,i,2400 + i);
	// RF_SETUP
	i = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	USART_printf(USART1,"[0x%02X] 0x%02X CONT_WAVE:%s PLL_LOCK:%s DataRate=",
			nRF24_REG_RF_SETUP,
			i,
			(i & 0x80) ? "ON" : "OFF",
			(i & 0x80) ? "ON" : "OFF"
		);
	switch ((i & 0x28) >> 3) {
		case 0x00:
			USART_printf(USART1,"1M");
			break;
		case 0x01:
			USART_printf(USART1,"2M");
			break;
		case 0x04:
			USART_printf(USART1,"250k");
			break;
		default:
			USART_printf(USART1,"???");
			break;
	}
	USART_printf(USART1,"pbs RF_PWR=");
	switch ((i & 0x06) >> 1) {
		case 0x00:
			USART_printf(USART1,"-18");
			break;
		case 0x01:
			USART_printf(USART1,"-12");
			break;
		case 0x02:
			USART_printf(USART1,"-6");
			break;
		case 0x03:
			USART_printf(USART1,"0");
			break;
		default:
			USART_printf(USART1,"???");
			break;
	}
	USART_printf(USART1,"dBm\r\n");
	// STATUS
	i = nRF24_ReadReg(nRF24_REG_STATUS);
	USART_printf(USART1,"[0x%02X] 0x%02X IRQ:%03b RX_PIPE:%u TX_FULL:%s\r\n",
			nRF24_REG_STATUS,
			i,
			(i & 0x70) >> 4,
			(i & 0x0E) >> 1,
			(i & 0x01) ? "YES" : "NO"
		);
	// OBSERVE_TX
	i = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
	USART_printf(USART1,"[0x%02X] 0x%02X PLOS_CNT=%u ARC_CNT=%u\r\n",nRF24_REG_OBSERVE_TX,i,i >> 4,i & 0x0F);
	// RPD
	i = nRF24_ReadReg(nRF24_REG_RPD);
	USART_printf(USART1,"[0x%02X] 0x%02X RPD=%s\r\n",nRF24_REG_RPD,i,(i & 0x01) ? "YES" : "NO");
	// RX_ADDR_P0
	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P0,buf,aw);
	USART_printf(USART1,"[0x%02X] RX_ADDR_P0 \"",nRF24_REG_RX_ADDR_P0);
	for (i = 0; i < aw; i++) USART_printf(USART1,"%c",buf[i]);
	USART_printf(USART1,"\"\r\n");
	// RX_ADDR_P1
	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P1,buf,aw);
	USART_printf(USART1,"[0x%02X] RX_ADDR_P1 \"",nRF24_REG_RX_ADDR_P1);
	for (i = 0; i < aw; i++) USART_printf(USART1,"%c",buf[i]);
	USART_printf(USART1,"\"\r\n");
	// RX_ADDR_P2
	USART_printf(USART1,"[0x%02X] RX_ADDR_P2 \"",nRF24_REG_RX_ADDR_P2);
	for (i = 0; i < aw - 1; i++) USART_printf(USART1,"%c",buf[i]);
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P2);
	USART_printf(USART1,"%c\"\r\n",i);
	// RX_ADDR_P3
	USART_printf(USART1,"[0x%02X] RX_ADDR_P3 \"",nRF24_REG_RX_ADDR_P3);
	for (i = 0; i < aw - 1; i++) USART_printf(USART1,"%c",buf[i]);
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P3);
	USART_printf(USART1,"%c\"\r\n",i);
	// RX_ADDR_P4
	USART_printf(USART1,"[0x%02X] RX_ADDR_P4 \"",nRF24_REG_RX_ADDR_P4);
	for (i = 0; i < aw - 1; i++) USART_printf(USART1,"%c",buf[i]);
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P4);
	USART_printf(USART1,"%c\"\r\n",i);
	// RX_ADDR_P5
	USART_printf(USART1,"[0x%02X] RX_ADDR_P5 \"",nRF24_REG_RX_ADDR_P5);
	for (i = 0; i < aw - 1; i++) USART_printf(USART1,"%c",buf[i]);
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P5);
	USART_printf(USART1,"%c\"\r\n",i);
	// TX_ADDR
	nRF24_ReadMBReg(nRF24_REG_TX_ADDR,buf,aw);
	USART_printf(USART1,"[0x%02X] TX_ADDR \"",nRF24_REG_TX_ADDR);
	for (i = 0; i < aw; i++) USART_printf(USART1,"%c",buf[i]);
	USART_printf(USART1,"\"\r\n");
	// RX_PW_P0
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P0);
	USART_printf(USART1,"[0x%02X] RX_PW_P0=%u\r\n",nRF24_REG_RX_PW_P0,i);
	// RX_PW_P1
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P1);
	USART_printf(USART1,"[0x%02X] RX_PW_P1=%u\r\n",nRF24_REG_RX_PW_P1,i);
	// RX_PW_P2
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P2);
	USART_printf(USART1,"[0x%02X] RX_PW_P2=%u\r\n",nRF24_REG_RX_PW_P2,i);
	// RX_PW_P3
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P3);
	USART_printf(USART1,"[0x%02X] RX_PW_P3=%u\r\n",nRF24_REG_RX_PW_P3,i);
	// RX_PW_P4
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P4);
	USART_printf(USART1,"[0x%02X] RX_PW_P4=%u\r\n",nRF24_REG_RX_PW_P4,i);
	// RX_PW_P5
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P5);
	USART_printf(USART1,"[0x%02X] RX_PW_P5=%u\r\n",nRF24_REG_RX_PW_P5,i);
}

*/
