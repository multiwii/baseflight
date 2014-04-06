/*
 * SphereCalibration.c
 *
 *  Created on: 06.02.2014
 *      Author: bubi-007
 */

#include "stdint.h"
#include "math.h"
#include "SphereFit.h"
#include "SphereCalibration.h"

#define maxSamples	700
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

static uint16_t sampleSToTake = 0;
static uint16_t calibrationCounter = 0;
static uint16_t population[2][3];

static float d[maxSamples][3];
static float sphereOrigin[3];
static float sphereRadius;

float calculateVariance(uint16_t axis, float *mean_val);
float calculateBiasCorrectedVariance(uint16_t axis, float *mean_val);
float getStandardDeviation(uint16_t axis, float *mean_val);
float getSampleStandardDeviation(uint16_t axis, float *mean_val);

void initCalibration(int samples)
{
    sampleSToTake = samples;
    if (sampleSToTake > maxSamples) {
        sampleSToTake = maxSamples;
    }
    calibrationCounter = 0;
}

void addSample(int16_t samples[3])
{
    if (calibrationCounter < sampleSToTake) {
        d[calibrationCounter][XAXIS] = (float)samples[XAXIS];
        d[calibrationCounter][YAXIS] = (float)samples[YAXIS];
        d[calibrationCounter][ZAXIS] = (float)samples[ZAXIS];

        calibrationCounter++;
    }
}

void spherefitCalibration(float calibrationValues[3], float variance[3], float mean_values[3])
{
    sphereFit(d, calibrationCounter, 100, 0.0f, population, sphereOrigin, &sphereRadius);

    calibrationValues[XAXIS] = sphereOrigin[XAXIS];
    calibrationValues[YAXIS] = sphereOrigin[YAXIS];
    calibrationValues[ZAXIS] = sphereOrigin[ZAXIS];

    // calculate the variance
    variance[0] = calculateBiasCorrectedVariance(0, &mean_values[0]);
    variance[1] = calculateBiasCorrectedVariance(1, &mean_values[1]);
    variance[2] = calculateBiasCorrectedVariance(2, &mean_values[2]);
}

// http://www.softwareandfinance.com/CPP/MeanVarianceStdDevi.html
double calculateMean(uint16_t axis)
{
    int i;
    double sum = 0;

    for (i = 0; i < maxSamples; i++) {
        sum += d[i][axis];
    }

    return (sum / maxSamples);
}

float calculateVariance(uint16_t axis, float *mean_val)
{
    int i;
    double mean = calculateMean(axis);
    double temp = 0;

    *mean_val = (float)mean;
    for (i = 0; i < maxSamples; i++) {
        temp += (d[i][axis] - mean) * (d[i][axis] - mean);
    }
    return (float)temp / maxSamples;
}

float calculateBiasCorrectedVariance(uint16_t axis, float *mean_val)
{
    int i;
    double mean = calculateMean(axis);
    double temp = 0;

    *mean_val = (float)mean;
    for (i = 0; i < maxSamples; i++) {
        temp += (d[i][axis] - mean) * (d[i][axis] - mean);
    }
    return (float)temp / (maxSamples - 1);
}

float getStandardDeviation(uint16_t axis, float *mean_val)
{
    return sqrtf(calculateVariance(axis, mean_val));
}

float getSampleStandardDeviation(uint16_t axis, float *mean_val)
{
    return sqrtf(calculateBiasCorrectedVariance(axis, mean_val));
}

