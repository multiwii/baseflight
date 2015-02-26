#include "board.h"
#include "mw.h"

// Driver for IBUS (Flysky) receiver using UART2

#define IBUS_BUFFSIZE 32
#define IBUS_MAX_CHANNEL 8
#define IBUS_SYNCBYTE 0x20

static bool ibusFrameDone = false;
static void ibusDataReceive(uint16_t c);
static uint16_t ibusReadRawRC(uint8_t chan);

static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

void ibusInit(rcReadRawDataPtr *callback)
{
    core.rcvrport = uartOpen(USART2, ibusDataReceive, 115200, MODE_RX);

    if (callback)
        *callback = ibusReadRawRC;
}

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };

// Receive ISR callback
static void ibusDataReceive(uint16_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > 3000)
        ibusFramePosition = 0;

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0 && c != IBUS_SYNCBYTE)
        return;

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == IBUS_BUFFSIZE - 1) {
        ibusFrameDone = true;
        failsafeCnt = 0;   // clear FailSafe counter
    } else {
        ibusFramePosition++;
    }
}

bool ibusFrameComplete(void)
{
    uint8_t i;
    uint16_t chksum, rxsum;

    if (ibusFrameDone) {
        ibusFrameDone = false;

        chksum = 0xFFFF;

        for (i = 0; i < 30; i++)
            chksum -= ibus[i];

        rxsum = ibus[30] + (ibus[31] << 8);

        if (chksum == rxsum) {
            ibusChannelData[0] = (ibus[ 3] << 8) + ibus[ 2];
            ibusChannelData[1] = (ibus[ 5] << 8) + ibus[ 4];
            ibusChannelData[2] = (ibus[ 7] << 8) + ibus[ 6];
            ibusChannelData[3] = (ibus[ 9] << 8) + ibus[ 8];
            ibusChannelData[4] = (ibus[11] << 8) + ibus[10];
            ibusChannelData[5] = (ibus[13] << 8) + ibus[12];
            ibusChannelData[6] = (ibus[15] << 8) + ibus[14];
            ibusChannelData[7] = (ibus[17] << 8) + ibus[16];
            return true;
        }
    }
    return false;
}

static uint16_t ibusReadRawRC(uint8_t chan)
{
    return ibusChannelData[mcfg.rcmap[chan]];
}
