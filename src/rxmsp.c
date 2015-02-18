/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#include "board.h"
#include "mw.h"

static bool rxMspFrameDone = false;
static uint16_t mspReadRawRC(uint8_t chan);

static uint16_t mspReadRawRC(uint8_t chan)
{
    return rcData[chan];
}

void mspFrameRecieve(void)
{
    rxMspFrameDone = true;
}

bool mspFrameComplete(void)
{
    if (rxMspFrameDone) {
        failsafeCnt = 0; // clear FailSafe counter
        rxMspFrameDone = false;
        return true;
    }
    return false;
}

void mspInit(rcReadRawDataPtr *callback)
{
    if (callback)
        *callback = mspReadRawRC;
}
