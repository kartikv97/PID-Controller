


/**
 * @copyright (c) 2020, Kartik Venkat, Nidhi Bhojak
 *
 * @file test.cpp
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
 * This is a test function to test the methods of the PID controller implementation.
 */

#include <gtest/gtest.h>
#include "lib.hpp"

/**
 * @brief This Test function aims to test the calculateCurrentError method of the
 *        ComputePID class.
 * @param PID_Test2 : - Test Name
 * @param errorCalculationTest : - Test description.
 * @return none
 */
TEST(PID_Test1, errorCalculationTest) {
  double kp = 10;
  double ki = 2;
  double kd = 5;

  ComputePID obj(kp, ki, kd);

  double TargetVelocity = 25;
  double ActualVelocity = 23;
  double CurrentError;

  CurrentError = obj.calculateCurrentError(TargetVelocity, ActualVelocity);

  EXPECT_EQ(CurrentError, 2);
}

/**
 * @brief This Test function aims to test the calculateAccumulatedError method of the
 *        ComputePID class.
 * @param PID_Test3 : - Test Name
 * @param computeMethodTest : - Test description.
 * @return none
 */
TEST(PID_Test2, accumulatedErrorTest) {
  double kp = 10;
  double ki = 2;
  double kd = 5;

  ComputePID obj(kp, ki, kd);

  double TargetVelocity = 25;
  double ActualVelocity = 23;
  double CurrentError;
  double AccumulatedError;

  std::vector <double> AccumulatedErrors = { 7, 5, 3 };

  CurrentError = obj.calculateCurrentError(TargetVelocity, ActualVelocity);

  AccumulatedError = obj.calculateAccumulatedError(CurrentError, AccumulatedErrors);

  EXPECT_EQ(CurrentError, 2);
  EXPECT_EQ(AccumulatedError, 17);
}

/**
 * @brief This Test function aims to test the calculatePID method of the
 *        ComputePID class.
 * @param PID_Test3 : - Test Name
 * @param computeMethodTest : - Test description.
 * @return none
 */
TEST(PID_Test3, computeMethodTest) {
  double kp = 10;
  double ki = 2;
  double kd = 5;

  ComputePID obj(kp, ki, kd);

  double TargetVelocity = 25;
  double ActualVelocity = 23;
  double PreviousError = 3;
  double CurrentError;
  double PID_Output;
  double AccumulatedError;

  std::vector <double> AccumulatedErrors = { 7, 5, 3 };

  CurrentError = obj.calculateCurrentError(TargetVelocity, ActualVelocity);

  AccumulatedError = obj.calculateAccumulatedError(CurrentError, AccumulatedErrors);

  PID_Output = obj.calculatePID(CurrentError, PreviousError, AccumulatedError);

  EXPECT_EQ(CurrentError, 2);
  EXPECT_EQ(AccumulatedError, 17);
  EXPECT_EQ(PID_Output, 49);

}