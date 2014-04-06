/*
 * MagCal.h
 *
 *  Created on: 06.02.2014
 *      Author: bubi-007
 */

#ifndef MAGCAL_H_
#define MAGCAL_H_

uint16_t sphereFit(float d[][3], uint16_t N, uint16_t MaxIterations, float Err,
		uint16_t Population[][3], float SphereOrigin[], float * SphereRadius);


#endif /* MAGCAL_H_ */
