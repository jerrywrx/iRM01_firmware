/******************************************************************************
 * @file     controller_functions.h
 * @brief    Public header file for CMSIS DSP Library
 * @version  V1.10.0
 * @date     08 July 2021
 * Target Processor: Cortex-M and Cortex-A cores
 ******************************************************************************/
/*
 * Copyright (c) 2010-2020 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 
#ifndef _CONTROLLER_FUNCTIONS_H_
#define _CONTROLLER_FUNCTIONS_H_

#include "arm_math_types.h"

#ifdef   __cplusplus
extern "C"
{
#endif

  /**
   * @brief Macros required for SINE and COSINE Controller functions
   */

#define CONTROLLER_Q31_SHIFT  (32 - 9)
  /* 1.31(q31) Fixed value of 2/360 */
  /* -1 to +1 is divided into 360 values so total spacing is (2/360) */
#define INPUT_SPACING         0xB60B61
  
/**
 * @defgroup groupController Controller Functions
 */


/**
  @ingroup groupController
 */

/**
  @defgroup SinCos Sine Cosine

  Computes the trigonometric sine and cosine values using a combination of table lookup
  and linear interpolation.
  There are separate functions for Q31 and floating-point data types.
  The input to the floating-point version is in degrees while the
  fixed-point Q31 have a scaled input with the range
  [-1 0.9999] mapping to [-180 +180] degrees.

  The floating point function also allows values that are out of the usual range. When this happens, the function will
  take extra time to adjust the input value to the range of [-180 180].

  The result is accurate to 5 digits after the decimal point.

  The implementation is based on table lookup using 360 values together with linear interpolation.
  The steps used are:
   -# Calculation of the nearest integer table index.
   -# Compute the fractional portion (fract) of the input.
   -# Fetch the value corresponding to \c index from sine table to \c y0 and also value from \c index+1 to \c y1.
   -# Sine value is computed as <code> *psinVal = y0 + (fract * (y1 - y0))</code>.
   -# Fetch the value corresponding to \c index from cosine table to \c y0 and also value from \c index+1 to \c y1.
   -# Cosine value is computed as <code> *pcosVal = y0 + (fract * (y1 - y0))</code>.
 */

/**
  @ingroup groupController
 */
  
/**
   * @defgroup PID PID Motor Control
   *
   * A Proportional Integral Derivative (PID) controller is a generic feedback control
   * loop mechanism widely used in industrial control systems.
   * A PID controller is the most commonly used type of feedback controller.
   *
   * This set of functions implements (PID) controllers
   * for Q15, Q31, and floating-point data types.  The functions operate on a single sample
   * of data and each call to the function returns a single processed value.
   * <code>S</code> points to an instance of the PID control data structure.  <code>in</code>
   * is the input sample value. The functions return the output value.
   *
   * \par Algorithm:
   * <pre>
   *    y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
   *    A0 = Kp + Ki + Kd
   *    A1 = (-Kp ) - (2 * Kd )
   *    A2 = Kd
   * </pre>
   *
   * \par
   * where \c Kp is proportional constant, \c Ki is Integral constant and \c Kd is Derivative constant
   *
   * \par
   * \image html PID.gif "Proportional Integral Derivative Controller"
   *
   * \par
   * The PID controller calculates an "error" value as the difference between
   * the measured output and the reference input.
   * The controller attempts to minimize the error by adjusting the process control inputs.
   * The proportional value determines the reaction to the current error,
   * the integral value determines the reaction based on the sum of recent errors,
   * and the derivative value determines the reaction based on the rate at which the error has been changing.
   *
   * \par Instance Structure
   * The Gains A0, A1, A2 and state variables for a PID controller are stored together in an instance data structure.
   * A separate instance structure must be defined for each PID Controller.
   * There are separate instance structure declarations for each of the 3 supported data types.
   *
   * \par Reset Functions
   * There is also an associated reset function for each data type which clears the state array.
   *
   * \par Initialization Functions
   * There is also an associated initialization function for each data type.
   * The initialization function performs the following operations:
   * - Initializes the Gains A0, A1, A2 from Kp,Ki, Kd gains.
   * - Zeros out the values in the state buffer.
   *
   * \par
   * Instance structure cannot be placed into a const data section and it is recommended to use the initialization function.
   *
   * \par Fixed-Point Behavior
   * Care must be taken when using the fixed-point versions of the PID Controller functions.
   * In particular, the overflow and saturation behavior of the accumulator used in each function must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */

  /**
   * @ingroup PID
   * @brief Instance structure for the floating-point PID Control.
   */
  typedef struct
  {
          float32_t A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
          float32_t A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
          float32_t A2;          /**< The derived gain, A2 = Kd . */
          float32_t state[3];    /**< The state array of length 3. */
          float32_t Kp;          /**< The proportional gain. */
          float32_t Ki;          /**< The integral gain. */
          float32_t Kd;          /**< The derivative gain. */
  } arm_pid_instance_f32;


  /**
   * @brief  Initialization function for the floating-point PID Control.
   * @param[in,out] S               points to an instance of the PID structure.
   * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
   */
  void arm_pid_init_f32(
        arm_pid_instance_f32 * S,
        int32_t resetStateFlag);


  /**
   * @ingroup PID
   * @brief         Process function for the floating-point PID Control.
   * @param[in,out] S   is an instance of the floating-point PID Control structure
   * @param[in]     in  input sample to process
   * @return        processed output sample.
   */
  __STATIC_FORCEINLINE float32_t arm_pid_f32(
  arm_pid_instance_f32 * S,
  float32_t in)
  {
    float32_t out;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

    /* Update state */
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    /* return to application */
    return (out);

  }


  
#ifdef   __cplusplus
}
#endif

#endif /* ifndef _CONTROLLER_FUNCTIONS_H_ */
