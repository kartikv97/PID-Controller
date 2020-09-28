


/**
 * @copyright (c) 2020, Kartik Venkat, Nidhi Bhojak
 *
 * @file PID.cpp
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
 * This file defines the methods declared in lib.hpp
 */
#include <iostream>
#include "lib.hpp"
ComputePID::ComputePID(double P_gain, double I_gain, double D_gain) {
  kp = P_gain;
  ki = I_gain;
  kd = D_gain;
}

double ComputePID::calculateCurrentError(double TargetSetPoint,
                                         double ActualVelocity) {
  double current_error;
  current_error = TargetSetPoint - ActualVelocity;
  return current_error;
}

double ComputePID::calculateAccumulatedError(
    double CurrentError, const std::vector<double> &AccumulatedErrors) {
  double acc_error;
  double vec_error_sum { 0 };
  for (auto vec_elem : AccumulatedErrors) {
    vec_error_sum += vec_elem;
  }
  acc_error = CurrentError + vec_error_sum;
  return acc_error;
}

double ComputePID::calculatePID(double CurrentError, double PreviousError,
  double AccumulatedError) {
  /*
   P = kp * current error
   I = ki * Accumulated error
   D = kd * current error - previous error
   */
  double gain;
  gain = CurrentError * kp + AccumulatedError * ki
      + (CurrentError - PreviousError) * kd;
  return gain;
}





