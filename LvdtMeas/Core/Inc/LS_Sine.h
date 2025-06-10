/*
 * LS_Sine.h
 *
 *  Created on: May 11, 2025
 *      Author: Elias
 */

#ifndef INC_LS_SINE_H_
#define INC_LS_SINE_H_

#include "main.h"
#include "nml.h"
#include <math.h>


void ls_sine_calcSine(uint16_t * signal, float dw, int nrPoints, float * amp, float * phase, float * offset);
void ls_sine_create_matrices(float MtM[3][3], float MtY[3], float dw, float* y, int nrPoints);
int ls_sine_inv_matrix(float Mat[3][3], float MatInv[3][3]);


#endif /* INC_LS_SINE_H_ */
