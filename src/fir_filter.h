/*
 * fir_filter.h
 *
 *  Created on: 25.11.2012
 *      Author: bubi-007
 */

#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

void accFilterStep(int16_t acc[3]);
void gyroFilterStep(int16_t gyros[3]);



#endif /* FIR_FILTER_H_ */
