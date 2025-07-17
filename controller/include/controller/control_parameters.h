#pragma once

/*
 *
 * Control parameters for the Adaptive Impedance Controller
 * Some of the following parameters are defined as constants, yet their value may be changed at runtine
 * to further customize the response of the controller. 
 * 
 * The values find below may be tuned to adjust the response of the controller.
 * 
 */

/// @brief Integral error threshold 
float integral_error_th = 10000.0f;

/// @brief Gain for the integral error
float Ki[2] = {2.5f, 2.5f};

/// @brief Impedance Parameters
//float Mim[2] = {1.0f, 1.0f};
//float Kim[2] = {15.0f, 15.0f};
//float Dim[2] = {10.0f, 10.0f};

/// @brief Impedance Parameters (Single Axis)
float Mim = 0.001472069;
float Kim = 0.004f;
float Dim = 1.1f;

//float Mim = 0.1;
//float Kim = 0.1;
//float Dim = 0.1f;



/// @brief Trajectory Parameters

float ca = 6.0f;



/// @brief Initial Conditions








/// @brief Number of cycles after which we reset the integral error
unsigned short integral_error_reset = 200;
