


/**
 * @copyright (c) 2020, Kartik Venkat, Nidhi Bhojak
 *
 * @file PID.cpp
 *
 * @authors
 * Kartik Venkat (kartikv97) \n
 * Nidhi Bhojak (nbhojak07) \n
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
#include "lib.hpp"


ComputePID::ComputePID(double P_gain, double I_gain, double D_gain) {
  kp = P_gain;
  ki = I_gain;
  kd = D_gain;
}

double ComputePID::calculateCurrentError(double TargetSetPoint, double ActualVelocity) {
  return 0;
}

double ComputePID::calculateAccumulatedError(double CurrentError, std::vector<double> AccumulatedErrors) {
  return 0;
}

double ComputePID::calculatePID(double CurrentError,
                                double PreviousError,
                                double AccumulatedError) {
  /*
  P = kp * current error
  I = ki * Accumulated error
  D = kd * current error - previous error
  */
  return 0;
}





