#ifndef __MATH_DRV_H__
#define __MATH_DRV_H__

#include "main.h"

void Calculate_Sin_Cos(float angle, float *sinval, float *cosval);
void Amplitude_Limit(float *input, float min, float max);
float Value_normalize(float angle);
void Value_Correct(float *angle, float error);
#endif
