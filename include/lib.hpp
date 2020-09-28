
/**
 * @copyright (c) 2020, Kartik Venkat, Nidhi Bhojak
 *
 * @file lib.hpp
 *
 * @authors
 * Part 1:
 * Kartik Venkat (kartikv97) ---- Driver \n
 * Nidhi Bhojak (nbhojak07)  ---- Navigator\n
 *
 * @version 1.0
 *
 * @section LICENSE
 *
 * BSD 3-Clause License
 *
 *
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file declares a class ("ComputePID") that is used to generate a new velocity given
 * a target set-point and actual velocity.
 */

#pragma once

#include <iostream>
#include <vector>

class ComputePID{
 private:
  double kp{ 0 };
  double ki{ 0 };
  double kd{ 0 };


 public:
  /**
   *  @brief Constructor for the class computePID.
   *  @param kp : - Proportional Gain.
   *  @param ki : - Integral Gain.
   *  @param kd : - Differential Gain.
   *  @return none
   */
  explicit ComputePID(double kp, double ki, double kd);

  /**
   * @brief A function to compute the current error by comparing
   *        the target velocity and the velocity from the sensor.
   * @param TargetSetPoint : - The target velocity for the system.
   * @param ActualVelocity : - The actual velocity of the system.
   * @return <current error of the system>                           // EDIT this..
   */
  double calculateCurrentError(double TargetSetPoint,
                               double ActualVelocity);

  /**
   * @brief A function to compute the Accumulated error of the system.
   * @param CurrentError : - The current error of the system.
   * @param AccumulatedError : - Vector of previous errors.
   * @return <Accumulated error of the system>                      // EDIT this..
   */
  double calculateAccumulatedError(double  CurrentError,
    const std::vector<double >&AccumulatedErrors);

  /**
   * @brief A function to compute the PID output velocity.
   * @param CurrentError : - The current error of the system.
   * @param PreviousError : - The previous error of the system.
   * @param AccumulatedError : - Sum of all the errors of the system.
   * @return <Final PID output>                                     // EDIT this..
   */
  double calculatePID(double CurrentError, double PreviousError,
                      double AccumulatedError);
};






