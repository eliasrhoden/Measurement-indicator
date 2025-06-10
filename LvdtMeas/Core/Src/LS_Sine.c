/*
 * LS_Sine.c
 *
 *  Created on: May 11, 2025
 *      Author: Elias
 */

#include "LS_Sine.h"






void ls_sine_calcSine(uint16_t * signal, float dw, int nrPoints, float * amp, float * phase, float * offset){

	float MtM[3][3];
	float MtM_inv[3][3];
	float MtY[3];
	float y[NO_POINTS_ADC];
	int invOk;
	float th0,th1,th2;

	// Convert signal to float
	for(int i=0;i<NO_POINTS_ADC;i++){
		if(i>=nrPoints){
			break;
		}
		y[i] = (float) signal[i];
	}

	// Create matrices
	ls_sine_create_matrices(MtM,  MtY, dw, y, nrPoints);

	// Invert M.t@M
	invOk = ls_sine_inv_matrix(MtM, MtM_inv);

	if(invOk){
		// Compute (M.T@M)^-1 @ M.T@Y
		th0 = MtM_inv[0][0]*MtY[0] + MtM_inv[0][1]*MtY[1] +  MtM_inv[0][2]*MtY[2];
		th1 = MtM_inv[1][0]*MtY[0] + MtM_inv[1][1]*MtY[1] +  MtM_inv[1][2]*MtY[2];
		th2 = MtM_inv[2][0]*MtY[0] + MtM_inv[2][1]*MtY[1] +  MtM_inv[2][2]*MtY[2];

		*amp = sqrtf(th0*th0 + th1*th1);
		*phase = atan2f(th1, th0);
		*offset = th2;
	}else{
		// No inverse
		amp = -1;
		phase = 0;
		offset = 0;
	}

}



/*
    Computes the Matrix M.T@M and M.T@Y,
    where M = [[sin(dw*0), cos(dw*0), 1],
               [sin(dw*1), cos(dw*1), 1],
               [sin(dw*3), cos(dw*2), 1],
               ...]

*/

void ls_sine_create_matrices(float MtM[3][3], float MtY[3], float dw, float* y, int nrPoints){

    float m11 = 0;
    float m12 = 0;
    float m13 = 0;

    float m22 = 0;
    float m23 = 0 ;
    float m33 = nrPoints;

    float MtY1 = 0;
    float MtY2 = 0;
    float MtY3 = 0;

    for(int i=0;i<nrPoints;i++){

        float ti = dw*i;

        m11 += sinf(ti)*sinf(ti);
        m12 += sinf(ti)*cosf(ti);
        m13 += sinf(ti);

        m22 += cosf(ti)*cosf(ti);
        m23 += cosf(ti);

        MtY1 += sinf(ti)*y[i];
        MtY2 += cosf(ti)*y[i];
        MtY3 += y[i];

    }

    MtM[0][0] = m11;
    MtM[0][1] = m12;
    MtM[0][2] = m13;

    MtM[1][0] = m12;
    MtM[1][1] = m22;
    MtM[1][2] = m23;

    MtM[2][0] = m13;
    MtM[2][1] = m12;
    MtM[2][2] = m33;

    MtY[0] = MtY1;
    MtY[1] = MtY2;
    MtY[2] = MtY3;
}

/* Computes the inverse of a 3x3 matrix
    If no inverse exits, it returns zero.
    If an inverse exits, it returns one.
*/
int ls_sine_inv_matrix(float Mat[3][3], float MatInv[3][3]){

    float det = Mat[0][0] * (Mat[1][1] * Mat[2][2] - Mat[2][1] * Mat[1][2]) -
                Mat[0][1] * (Mat[1][0] * Mat[2][2] - Mat[1][2] * Mat[2][0]) +
                Mat[0][2] * (Mat[1][0] * Mat[2][1] - Mat[1][1] * Mat[2][0]);

    if (abs(det)<0.0001){
        return 0;
    }

    float invdet = 1 / det;

    MatInv[0][0] = (Mat[1][1] * Mat[2][2] - Mat[2][1] * Mat[1][2]) * invdet;
    MatInv[0][1] = (Mat[0][2] * Mat[2][1] - Mat[0][1] * Mat[2][2]) * invdet;
    MatInv[0][2] = (Mat[0][1] * Mat[1][2] - Mat[0][2] * Mat[1][1]) * invdet;
    MatInv[1][0] = (Mat[1][2] * Mat[2][0] - Mat[1][0] * Mat[2][2]) * invdet;
    MatInv[1][1] = (Mat[0][0] * Mat[2][2] - Mat[0][2] * Mat[2][0]) * invdet;
    MatInv[1][2] = (Mat[1][0] * Mat[0][2] - Mat[0][0] * Mat[1][2]) * invdet;
    MatInv[2][0] = (Mat[1][0] * Mat[2][1] - Mat[2][0] * Mat[1][1]) * invdet;
    MatInv[2][1] = (Mat[2][0] * Mat[0][1] - Mat[0][0] * Mat[2][1]) * invdet;
    MatInv[2][2] = (Mat[0][0] * Mat[1][1] - Mat[1][0] * Mat[0][1]) * invdet;

    return 1;

}





