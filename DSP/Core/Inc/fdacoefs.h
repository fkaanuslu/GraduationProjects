/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.10 and Signal Processing Toolbox 8.6.
 * Generated on: 24-Jan-2023 21:30:02
 */

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 24
 * Stable            : Yes
 * Linear Phase      : Yes (Type 2)
 */

/* This code generated by using filterDesigner tool in MATLAB */
// #include "tmwtypes.h"
/* 
 * Expected path to tmwtypes.h 
 * D:\MATLAB\MATLAB\extern\include\tmwtypes.h 
 */

/*You can just copy this part without adding the header file to main */
const int NUM_TAPS = 24;
const float32_t firCoeffs32[NUM_TAPS] = {
   0.001083533227666, 0.003775348456944, 0.006527502826968,  0.00434530488403,
  -0.008369651092956, -0.03089272458715, -0.05025621157586, -0.04394831977039,
   0.006296657726293,  0.09745578860733,   0.1990329152105,   0.2662537684149,
     0.2662537684149,   0.1990329152105,  0.09745578860733, 0.006296657726293,
   -0.04394831977039, -0.05025621157586, -0.03089272458715,-0.008369651092956,
    0.00434530488403, 0.006527502826968, 0.003775348456944, 0.001083533227666
};
// Delete the last coefficents because it repeats itself
