/*
 * SphereCalibration.h
 *
 *  Created on: 06.02.2014
 *      Author: bubi-007
 */

#ifndef SPHERECALIBRATION_H_
#define SPHERECALIBRATION_H_

void initCalibration(int sampleToTake);
void addSample(int16_t samples[3]);
void spherefitCalibration(float calibrationValues[3], float variance[3], float mean_values[3]);

#endif /* SPHERECALIBRATION_H_ */
