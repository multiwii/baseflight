/*
 * fir_filter.c
 *
 *  Created on: 25.11.2012
 *      Author & Copyright: bubi-007
 */

#include "stdint.h"
#include "fir_filter.h"


// stripped down being a fir filter
// http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
typedef struct
{
	float q;	// process noise covariance
	float r;	// measurement noise covariance
	float x;	// value
	float p;	// estimation error covariance
} fir_state;

// accelerometer
fir_state kax;
fir_state kay;
fir_state kaz;

// gyro
fir_state kgx;
fir_state kgy;
fir_state kgz;

void fir_filter_init(fir_state* state, float q, float r, float p, float intial_value)
{
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

// all the matrix stuff is missing
// no fun with inverted matrixes...
void fir_filter_update(fir_state* state, float measurement)
{

	// prediction update
	// omit x = x
	float k; // gain

	state->p = state->p + state->q;

	// measurement update
	k = state->p / (state->p + state->r);
	state->x = state->x + k * (measurement - state->x);
	state->p = (1 - k) * state->p;
}

void init_fir_filter_accel(float ax, float ay, float az)
{
#define Q 0.0625		// process noise covariance
#define	R 1.0			// measurement noise covariance
#define P 0.22			// estimation error covariance
	fir_filter_init(&kax, Q, R, P, ax);
	fir_filter_init(&kay, Q, R, P, ay);
	fir_filter_init(&kaz, Q, R, P, az);
#undef Q
#undef R
#undef P
}

void init_fir_filter_gyro(float gx, float gy, float gz)
{
#define Q 1.0 		// process noise covariance
#define	R 0.0625	// measurement noise covariance
#define P 0.22		// estimation error covariance
	fir_filter_init(&kgx, Q, R, P, gx);
	fir_filter_init(&kgy, Q, R, P, gy);
	fir_filter_init(&kgz, Q, R, P, gz);
#undef Q
#undef R
#undef P
}

int16_t fir_filter_step(fir_state* state, int16_t value)
{
	float measurement = value;
	fir_filter_update(state, measurement);
	return (int16_t)(state->x + 0.5f);
}

void accFilterStep(int16_t accels[3])
{
	static int _init = 0;
	if (!_init)
	{
		_init = 1;
		init_fir_filter_accel(accels[0], accels[1], accels[2]);
	}
	else
	{
		accels[0] = fir_filter_step(&kax, accels[0]);
		accels[1] = fir_filter_step(&kay, accels[1]);
		accels[2] = fir_filter_step(&kaz, accels[2]);
	}
}

void gyroFilterStep(int16_t gyros[3])
{
	static int _init = 0;
	if (!_init)
	{
		_init = 1;
		init_fir_filter_gyro(gyros[0], gyros[1], gyros[2]);
	}
	else
	{
		gyros[0] = fir_filter_step(&kgx, gyros[0]);
		gyros[1] = fir_filter_step(&kgy, gyros[1]);
		gyros[2] = fir_filter_step(&kgz, gyros[2]);
	}
}
