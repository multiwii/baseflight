/*
 * blackbox.h
 *
 *      Author: Nicholas Sherlock
 */

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include <stdint.h>

#ifndef XYZ_AXIS_COUNT
#define XYZ_AXIS_COUNT 3
#endif

typedef struct blackboxValues_t {
	uint32_t time;

    int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

	int16_t rcCommand[4];
    int16_t gyroData[XYZ_AXIS_COUNT];
    int16_t accSmooth[XYZ_AXIS_COUNT];
	int16_t motor[MAX_MOTORS];
	int16_t servo[MAX_SERVOS];
	
    uint16_t vbatLatest;

#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
} blackboxValues_t;

void initBlackbox(void);
void handleBlackbox(void);
void startBlackbox(void);
void finishBlackbox(void);

#endif /* BLACKBOX_H_ */
