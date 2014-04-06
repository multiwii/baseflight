// Least squares fit a sphere to 3D data, ImaginaryZ's blog,
// Miscellaneous banter, Useful mathematics, game programming tools and the occasional kink or two.
// 22 April 2011.
// http: imaginaryz.blogspot.com.au/2011/04/least-squares-fit-sphere-to-3d-data.html

// Substantially rewritten for UAVXArm by Prof. G.K. Egan (C) 2012.

#include "stdint.h"
#include "math.h"
#include "SphereFit.h"

#define X 0
#define Y 1
#define Z 2
#define Sqr(x) ((x) * (x))

uint16_t sphereFit(float d[][3], uint16_t N, uint16_t MaxIterations, float Err,
		uint16_t Population[][3], float SphereOrigin[], float * SphereRadius)
{

	uint8_t c;
	uint16_t i, Iterations;
	float s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
	float x2sum[3], y2sum[3], z2sum[3];
	float xy_sum, xz_sum, yz_sum;
	float XY, XZ, YZ, X2Z, Y2X, Y2Z, Z2X, X2Y, Z2Y;
	float QS, QB, Q0, Q1, Q2;
	float R2, C[3], C2[3], Delta[3], Denom[3];
	float F0, F1, F2, F3, F4;
	float di2[3];
	float SizeR;

	for (c = X; c <= Z; c++)
	{
		s[c] = s2[c] = s3[c] = sum[c] = x2sum[c] = y2sum[c] = z2sum[c] = 0.0f;
		Population[0][c] = Population[1][c] = 0;
	}

	xy_sum = xz_sum = yz_sum = 0.0f;

	for (i = 0; i < N; i++)
	{

		for (c = X; c <= Z; c++)
		{
			di2[c] = Sqr(d[i][c]);
			s[c] += d[i][c];
			s2[c] += di2[c];
			s3[c] += di2[c] * d[i][c];
			Population[d[i][c] > 0.0f][c]++;
		}

		xy_sum += d[i][X] * d[i][Y];
		xz_sum += d[i][X] * d[i][Z];
		yz_sum += d[i][Y] * d[i][Z];

		x2sum[Y] += di2[X] * d[i][Y];
		x2sum[Z] += di2[X] * d[i][Z];

		y2sum[X] += di2[Y] * d[i][X];
		y2sum[Z] += di2[Y] * d[i][Z];

		z2sum[X] += di2[Z] * d[i][X];
		z2sum[Y] += di2[Z] * d[i][Y];
	}

	SizeR = 1.0f / (float) N;
	for (c = X; c <= Z; c++)
	{
		sum[c] = s[c] * SizeR; //sum( X[n] )
		sum2[c] = s2[c] * SizeR; //sum( X[n]^2 )
		sum3[c] = s3[c] * SizeR; //sum( X[n]^3 )
	}

	XY = xy_sum * SizeR; //sum( X[n] * Y[n] )
	XZ = xz_sum * SizeR; //sum( X[n] * Z[n] )
	YZ = yz_sum * SizeR; //sum( Y[n] * Z[n] )

	X2Y = x2sum[Y] * SizeR; //sum( X[n]^2 * Y[n] )
	X2Z = x2sum[Z] * SizeR; //sum( X[n]^2 * Z[n] )
	Y2X = y2sum[X] * SizeR; //sum( Y[n]^2 * X[n] )
	Y2Z = y2sum[Z] * SizeR; //sum( Y[n]^2 * Z[n] )
	Z2X = z2sum[X] * SizeR; //sum( Z[n]^2 * X[n] )
	Z2Y = z2sum[Y] * SizeR; //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	F0 = sum2[X] + sum2[Y] + sum2[Z];
	F1 = 0.5f * F0;
	F2 = -8.0f * (sum3[X] + Y2X + Z2X);
	F3 = -8.0f * (X2Y + sum3[Y] + Z2Y);
	F4 = -8.0f * (X2Z + Y2Z + sum3[Z]);

	for (c = X; c <= Z; c++)
	{
		C[c] = sum[c];
		C2[c] = Sqr(C[c]);
	}

	QS = C2[X] + C2[Y] + C2[Z];
	QB = -2.0f * (Sqr(C[X]) + Sqr(C[Y]) + Sqr(C[Z]));
	R2 = F0 + QB + QS;
	Q0 = 0.5f * (QS - R2);
	Q1 = F1 + Q0;
	Q2 = 8.0f * (QS - R2 + QB + F0);

	Iterations = 0;
	do
	{
		for (c = X; c <= Z; c++)
		{
			Denom[c] = Q2 + 16.0f * (C2[c] - 2.0f * C[c] * sum[c] + sum2[c]);
			if (Denom[c] == 0.0f)
			{
				Denom[c] = 1.0f;
			}
		}

		Delta[X] = -((F2 + 16.0f * (C[Y] * XY + C[Z] * XZ + sum[X] * (-C2[X]
				- Q0) + C[X] * (sum2[X] + Q1 - C[Z] * sum[Z] - C[Y] * sum[Y])))
				/ Denom[X]);
		Delta[Y] = -((F3 + 16.0f * (C[X] * XY + C[Z] * YZ + sum[Y] * (-C2[Y]
				- Q0) + C[Y] * (sum2[Y] + Q1 - C[X] * sum[X] - C[Z] * sum[Z])))
				/ Denom[Y]);
		Delta[Z] = -((F4 + 16.0f * (C[X] * XZ + C[Y] * YZ + sum[Z] * (-C2[Z]
				- Q0) + C[Z] * (sum2[Z] + Q1 - C[X] * sum[X] - C[Y] * sum[Y])))
				/ Denom[Z]);

		for (c = X; c <= Z; c++)
		{
			C[c] += Delta[c];
			C2[c] = Sqr(C[c]);
		}

		QS = C2[X] + C2[Y] + C2[Z];
		QB = -2.0f * (C[X] * sum[X] + C[Y] * sum[Y] + C[Z] * sum[Z]);
		R2 = F0 + QB + QS;
		Q0 = 0.5f * (QS - R2);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - R2 + QB + F0);

		Iterations++;
	} while ((Iterations < 50) || ((Iterations < MaxIterations)
		&& ((Sqr(Delta[X]) + Sqr(Delta[Y]) + Sqr(Delta[Z])) > Err)));

	for (c = X; c <= Z; c++)
		SphereOrigin[c] = C[c];

	*SphereRadius = sqrtf(R2);

	return (Iterations);
}
// SphereFit
