/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;
const real_t* od = in + 10;
/* Vector of auxiliary variables; number of elements: 8. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = (sin(xd[2]));
a[3] = (cos(xd[2]));
a[4] = (cos(xd[2]));
a[5] = (sin(xd[2]));
a[6] = (atan((((real_t)(1.2200000000000000e+00)*xd[3])-xd[5])));
a[7] = (((real_t)(2.0000000000000000e+05)*a[6])/xd[4]);

/* Compute outputs: */
out[0] = (((xd[4]*a[0])-(xd[5]*a[1]))/((real_t)(1.0000000000000000e+00)-(xd[1]*od[0])));
out[1] = ((xd[4]*a[2])+(xd[5]*a[3]));
out[2] = (xd[3]-((od[0]*((xd[4]*a[4])-(xd[5]*a[5])))/((real_t)(1.0000000000000000e+00)-(xd[1]*od[0]))));
out[3] = ((real_t)(9.0909090909090905e-03)*(((real_t)(1.2200000000000000e+00)*u[0])-((real_t)(1.2200000000000000e+00)*a[7])));
out[4] = ((real_t)(5.2631578947368420e-03)*u[1]);
out[5] = (((real_t)(5.2631578947368420e-03)*(u[0]+a[7]))-(xd[4]*xd[3]));
out[6] = u[2];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 10;
/* Vector of auxiliary variables; number of elements: 30. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)-(xd[1]*od[0])));
a[3] = (a[2]*a[2]);
a[4] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[5] = (cos(xd[2]));
a[6] = (cos(xd[2]));
a[7] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[8] = (sin(xd[2]));
a[9] = (cos(xd[2]));
a[10] = (cos(xd[2]));
a[11] = (sin(xd[2]));
a[12] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)-(xd[1]*od[0])));
a[13] = (a[12]*a[12]);
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[15] = (cos(xd[2]));
a[16] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow((((real_t)(1.2200000000000000e+00)*xd[3])-xd[5]),2))));
a[17] = ((real_t)(1.2200000000000000e+00)*a[16]);
a[18] = ((real_t)(1.0000000000000000e+00)/xd[4]);
a[19] = (((real_t)(2.0000000000000000e+05)*a[17])*a[18]);
a[20] = (atan((((real_t)(1.2200000000000000e+00)*xd[3])-xd[5])));
a[21] = (a[18]*a[18]);
a[22] = ((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+05)*a[20])*a[21]));
a[23] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[16]);
a[24] = (((real_t)(2.0000000000000000e+05)*a[23])*a[18]);
a[25] = ((real_t)(1.2200000000000000e+00)*a[16]);
a[26] = (((real_t)(2.0000000000000000e+05)*a[25])*a[18]);
a[27] = ((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+05)*a[20])*a[21]));
a[28] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[16]);
a[29] = (((real_t)(2.0000000000000000e+05)*a[28])*a[18]);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = ((real_t)(0.0000000000000000e+00)-((((xd[4]*a[0])-(xd[5]*a[1]))*((real_t)(0.0000000000000000e+00)-od[0]))*a[3]));
out[2] = (((xd[4]*a[4])-(xd[5]*a[5]))*a[2]);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (a[0]*a[2]);
out[5] = (((real_t)(0.0000000000000000e+00)-a[1])*a[2]);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = ((xd[4]*a[6])+(xd[5]*a[7]));
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = a[8];
out[15] = a[9];
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = ((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(((od[0]*((xd[4]*a[10])-(xd[5]*a[11])))*((real_t)(0.0000000000000000e+00)-od[0]))*a[13])));
out[22] = ((real_t)(0.0000000000000000e+00)-((od[0]*((xd[4]*a[14])-(xd[5]*a[15])))*a[12]));
out[23] = (real_t)(1.0000000000000000e+00);
out[24] = ((real_t)(0.0000000000000000e+00)-((od[0]*a[10])*a[12]));
out[25] = ((real_t)(0.0000000000000000e+00)-((od[0]*((real_t)(0.0000000000000000e+00)-a[11]))*a[12]));
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = ((real_t)(9.0909090909090905e-03)*((real_t)(0.0000000000000000e+00)-((real_t)(1.2200000000000000e+00)*a[19])));
out[34] = ((real_t)(9.0909090909090905e-03)*((real_t)(0.0000000000000000e+00)-((real_t)(1.2200000000000000e+00)*a[22])));
out[35] = ((real_t)(9.0909090909090905e-03)*((real_t)(0.0000000000000000e+00)-((real_t)(1.2200000000000000e+00)*a[24])));
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = ((real_t)(1.1090909090909091e-02));
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(5.2631578947368420e-03);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (((real_t)(5.2631578947368420e-03)*a[26])-xd[4]);
out[54] = (((real_t)(5.2631578947368420e-03)*a[27])-xd[3]);
out[55] = ((real_t)(5.2631578947368420e-03)*a[29]);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(5.2631578947368420e-03);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim7_triangular( real_t* const A, real_t* const b )
{

b[6] = b[6]/A[48];
b[5] -= + A[41]*b[6];
b[5] = b[5]/A[40];
b[4] -= + A[34]*b[6];
b[4] -= + A[33]*b[5];
b[4] = b[4]/A[32];
b[3] -= + A[27]*b[6];
b[3] -= + A[26]*b[5];
b[3] -= + A[25]*b[4];
b[3] = b[3]/A[24];
b[2] -= + A[20]*b[6];
b[2] -= + A[19]*b[5];
b[2] -= + A[18]*b[4];
b[2] -= + A[17]*b[3];
b[2] = b[2]/A[16];
b[1] -= + A[13]*b[6];
b[1] -= + A[12]*b[5];
b[1] -= + A[11]*b[4];
b[1] -= + A[10]*b[3];
b[1] -= + A[9]*b[2];
b[1] = b[1]/A[8];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim7_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 7; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (6); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*7+i]);
	for( j=(i+1); j < 7; j++ ) {
		temp = fabs(A[j*7+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 7; ++k)
{
	acadoWorkspace.rk_dim7_swap = A[i*7+k];
	A[i*7+k] = A[indexMax*7+k];
	A[indexMax*7+k] = acadoWorkspace.rk_dim7_swap;
}
	acadoWorkspace.rk_dim7_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim7_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*7+i];
	for( j=i+1; j < 7; j++ ) {
		A[j*7+i] = -A[j*7+i]/A[i*7+i];
		for( k=i+1; k < 7; k++ ) {
			A[j*7+k] += A[j*7+i] * A[i*7+k];
		}
		b[j] += A[j*7+i] * b[i];
	}
}
det *= A[48];
det = fabs(det);
acado_solve_dim7_triangular( A, b );
return det;
}

void acado_solve_dim7_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim7_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim7_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim7_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim7_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim7_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim7_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim7_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim7_bPerm[1] += A[7]*acadoWorkspace.rk_dim7_bPerm[0];

acadoWorkspace.rk_dim7_bPerm[2] += A[14]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[2] += A[15]*acadoWorkspace.rk_dim7_bPerm[1];

acadoWorkspace.rk_dim7_bPerm[3] += A[21]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[3] += A[22]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[3] += A[23]*acadoWorkspace.rk_dim7_bPerm[2];

acadoWorkspace.rk_dim7_bPerm[4] += A[28]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[4] += A[29]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[4] += A[30]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[4] += A[31]*acadoWorkspace.rk_dim7_bPerm[3];

acadoWorkspace.rk_dim7_bPerm[5] += A[35]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[5] += A[36]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[5] += A[37]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[5] += A[38]*acadoWorkspace.rk_dim7_bPerm[3];
acadoWorkspace.rk_dim7_bPerm[5] += A[39]*acadoWorkspace.rk_dim7_bPerm[4];

acadoWorkspace.rk_dim7_bPerm[6] += A[42]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[6] += A[43]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[6] += A[44]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[6] += A[45]*acadoWorkspace.rk_dim7_bPerm[3];
acadoWorkspace.rk_dim7_bPerm[6] += A[46]*acadoWorkspace.rk_dim7_bPerm[4];
acadoWorkspace.rk_dim7_bPerm[6] += A[47]*acadoWorkspace.rk_dim7_bPerm[5];


acado_solve_dim7_triangular( A, acadoWorkspace.rk_dim7_bPerm );
b[0] = acadoWorkspace.rk_dim7_bPerm[0];
b[1] = acadoWorkspace.rk_dim7_bPerm[1];
b[2] = acadoWorkspace.rk_dim7_bPerm[2];
b[3] = acadoWorkspace.rk_dim7_bPerm[3];
b[4] = acadoWorkspace.rk_dim7_bPerm[4];
b[5] = acadoWorkspace.rk_dim7_bPerm[5];
b[6] = acadoWorkspace.rk_dim7_bPerm[6];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 5.0000000000000001e-03 };


/* Fixed step size:0.01 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[7] = rk_eta[77];
acadoWorkspace.rk_xxx[8] = rk_eta[78];
acadoWorkspace.rk_xxx[9] = rk_eta[79];
acadoWorkspace.rk_xxx[10] = rk_eta[80];
acadoWorkspace.rk_xxx[11] = rk_eta[81];
acadoWorkspace.rk_xxx[12] = rk_eta[82];
acadoWorkspace.rk_xxx[13] = rk_eta[83];
acadoWorkspace.rk_xxx[14] = rk_eta[84];

for (run = 0; run < 10; ++run)
{
if( run > 0 ) {
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 10] = rk_eta[i * 7 + 7];
acadoWorkspace.rk_diffsPrev2[i * 10 + 1] = rk_eta[i * 7 + 8];
acadoWorkspace.rk_diffsPrev2[i * 10 + 2] = rk_eta[i * 7 + 9];
acadoWorkspace.rk_diffsPrev2[i * 10 + 3] = rk_eta[i * 7 + 10];
acadoWorkspace.rk_diffsPrev2[i * 10 + 4] = rk_eta[i * 7 + 11];
acadoWorkspace.rk_diffsPrev2[i * 10 + 5] = rk_eta[i * 7 + 12];
acadoWorkspace.rk_diffsPrev2[i * 10 + 6] = rk_eta[i * 7 + 13];
acadoWorkspace.rk_diffsPrev2[i * 10 + 7] = rk_eta[i * 3 + 56];
acadoWorkspace.rk_diffsPrev2[i * 10 + 8] = rk_eta[i * 3 + 57];
acadoWorkspace.rk_diffsPrev2[i * 10 + 9] = rk_eta[i * 3 + 58];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 70 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
acadoWorkspace.rk_A[tmp_index1 * 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 6)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 7) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 7] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 7 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 7 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 7 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 7 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 7 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 7 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
}
det = acado_solve_dim7_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 7];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 7 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 7 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 7 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 7 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 7 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 7 + 6];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 7] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 7 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 7 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 7 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 7 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 7 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 7 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
}
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 7];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 7 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 7 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 7 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 7 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 7 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 7 + 6];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 70 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
acadoWorkspace.rk_A[tmp_index1 * 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 70) + (j * 10 + 6)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 7) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 7; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 7] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1)];
acadoWorkspace.rk_b[i * 7 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 10)];
acadoWorkspace.rk_b[i * 7 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 20)];
acadoWorkspace.rk_b[i * 7 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 30)];
acadoWorkspace.rk_b[i * 7 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 40)];
acadoWorkspace.rk_b[i * 7 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 50)];
acadoWorkspace.rk_b[i * 7 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (run1 + 60)];
}
if( 0 == run1 ) {
det = acado_solve_dim7_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
}
 else {
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 7];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 7 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 7 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 7 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 7 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 7 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 10) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 10) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)1.0000000000000000e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = (i * 7) + (j);
tmp_index2 = (run1) + (j * 10);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 70) + (tmp_index2 + 7)];
}
}
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 7];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 7 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 7 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 7 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 7 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 7 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 10) + (run1 + 7)] = + acadoWorkspace.rk_diffK[i]*(real_t)1.0000000000000000e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)1.0000000000000000e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)1.0000000000000000e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)1.0000000000000000e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[3]*(real_t)1.0000000000000000e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[4]*(real_t)1.0000000000000000e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[5]*(real_t)1.0000000000000000e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[6]*(real_t)1.0000000000000000e-02;
if( run == 0 ) {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = acadoWorkspace.rk_diffsNew2[(i * 10) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 56] = acadoWorkspace.rk_diffsNew2[(i * 10) + (j + 7)];
}
}
}
else {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = + acadoWorkspace.rk_diffsNew2[i * 10]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 1]*acadoWorkspace.rk_diffsPrev2[j + 10];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 2]*acadoWorkspace.rk_diffsPrev2[j + 20];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 3]*acadoWorkspace.rk_diffsPrev2[j + 30];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 4]*acadoWorkspace.rk_diffsPrev2[j + 40];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 5]*acadoWorkspace.rk_diffsPrev2[j + 50];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 10 + 6]*acadoWorkspace.rk_diffsPrev2[j + 60];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 56] = acadoWorkspace.rk_diffsNew2[(i * 10) + (j + 7)];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10]*acadoWorkspace.rk_diffsPrev2[j + 7];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 1]*acadoWorkspace.rk_diffsPrev2[j + 17];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 2]*acadoWorkspace.rk_diffsPrev2[j + 27];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 3]*acadoWorkspace.rk_diffsPrev2[j + 37];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 4]*acadoWorkspace.rk_diffsPrev2[j + 47];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 5]*acadoWorkspace.rk_diffsPrev2[j + 57];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 10 + 6]*acadoWorkspace.rk_diffsPrev2[j + 67];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000001e-01;
}
for (i = 0; i < 7; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



