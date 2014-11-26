/*
 * drv_softserial.h
 *
 *  Created on: 23 Aug 2013
 *      Author: Hydra
 */

#pragma once

#define SOFT_SERIAL_BUFFER_SIZE 256
// Max baud rate of current soft serial implementation
#define SOFT_SERIAL_MAX_BAUD_RATE 19200

typedef struct softSerial_s {
    serialPort_t port;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[SOFT_SERIAL_BUFFER_SIZE];

    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[SOFT_SERIAL_BUFFER_SIZE];
    
    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxEdge;

    uint8_t          isTransmittingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint8_t          isInverted;

    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;
} softSerial_t;

extern timerHardware_t* serialTimerHardware;
extern softSerial_t softSerialPorts[];

extern const struct serialPortVTable softSerialVTable[];

void setupSoftSerialPrimary(uint32_t baud, uint8_t inverted);
void setupSoftSerialSecondary(uint8_t inverted);

// serialPort API
void softSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t softSerialTotalBytesWaiting(serialPort_t *instance);
uint8_t softSerialReadByte(serialPort_t *instance);
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isSoftSerialTransmitBufferEmpty(serialPort_t *s);

