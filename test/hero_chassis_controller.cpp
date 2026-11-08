/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Zane Ji
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file hero_chassis_controller.cpp
 * @brief Unit tests of the mecanum wheel kinematics used by HeroChassisController.
 *
 * The kinematics of the controller are pure functions of the chassis geometry
 * and the wheel angular velocities. This file re-implements them with the same
 * expressions as src/hero_chassis_controller.cpp so that they can be verified
 * without starting Gazebo or ros_control.
 */

#include <cmath>

#include <gtest/gtest.h>

namespace hero_chassis_controller
{
namespace
{
/// @brief Wheel radius in meters, same as config/hero_chassis_controller.yaml.
constexpr double WHEEL_RADIUS = 0.07625;

/// @brief Distance between the front and back wheel centres in meters.
constexpr double TRACK_WIDTH_X = 0.4;

/// @brief Distance between the left and right wheel centres in meters.
constexpr double TRACK_WIDTH_Y = 0.4;

/// @brief Index of each wheel in the speed array, same order as wheel_joints_.
constexpr int LEFT_FRONT = 0;
constexpr int RIGHT_FRONT = 1;
constexpr int LEFT_BACK = 2;
constexpr int RIGHT_BACK = 3;

/// @brief Number of wheels of the chassis.
constexpr int WHEEL_COUNT = 4;

/// @brief Absolute tolerance for floating point comparisons.
constexpr double TOLERANCE = 1e-9;

/// @brief Chassis geometry shared by all test cases.
struct ChassisGeometry
{
  double wheel_radius_{ WHEEL_RADIUS };
  double track_width_x_{ TRACK_WIDTH_X };
  double track_width_y_{ TRACK_WIDTH_Y };
};

/**
 * @brief Inverse kinematics: chassis velocity to wheel angular velocities.
 *
 * Same expressions as src/hero_chassis_controller.cpp.
 *
 * @param geometry The chassis geometry.
 * @param cmd_vx Forward velocity in the chassis frame in m/s.
 * @param cmd_vy Left velocity in the chassis frame in m/s.
 * @param cmd_wz Yaw rate in the chassis frame in rad/s.
 * @param target_speeds Output array of WHEEL_COUNT wheel angular velocities.
 */
void computeWheelSpeeds(const ChassisGeometry& geometry, double cmd_vx, double cmd_vy, double cmd_wz,
                        double* target_speeds)
{
  const double L_sum = geometry.track_width_x_ / 2 + geometry.track_width_y_ / 2;

  target_speeds[LEFT_FRONT] = (cmd_vx - cmd_vy - L_sum * cmd_wz) / geometry.wheel_radius_;
  target_speeds[RIGHT_FRONT] = (cmd_vx + cmd_vy + L_sum * cmd_wz) / geometry.wheel_radius_;
  target_speeds[LEFT_BACK] = (cmd_vx + cmd_vy - L_sum * cmd_wz) / geometry.wheel_radius_;
  target_speeds[RIGHT_BACK] = (cmd_vx - cmd_vy + L_sum * cmd_wz) / geometry.wheel_radius_;
}

/**
 * @brief Forward kinematics: wheel angular velocities to chassis velocity.
 *
 * Same expressions as src/hero_chassis_controller.cpp.
 *
 * @param geometry The chassis geometry.
 * @param target_speeds Measured wheel angular velocities of WHEEL_COUNT wheels.
 * @param vx_forward Output forward velocity in m/s.
 * @param vy_forward Output left velocity in m/s.
 * @param wz_forward Output yaw rate in rad/s.
 */
void computeChassisVelocity(const ChassisGeometry& geometry, const double* target_speeds, double* vx_forward,
                            double* vy_forward, double* wz_forward)
{
  const double L_sum = geometry.track_width_x_ / 2 + geometry.track_width_y_ / 2;

  const double w_lf = target_speeds[LEFT_FRONT];
  const double w_rf = target_speeds[RIGHT_FRONT];
  const double w_lb = target_speeds[LEFT_BACK];
  const double w_rb = target_speeds[RIGHT_BACK];

  *vx_forward = geometry.wheel_radius_ * (w_lf + w_rf + w_lb + w_rb) / 4.0;
  *vy_forward = geometry.wheel_radius_ * (-w_lf + w_rf + w_lb - w_rb) / 4.0;
  *wz_forward = geometry.wheel_radius_ * (-w_lf + w_rf - w_lb + w_rb) / (4.0 * L_sum);
}

}  // namespace

/**
 * @brief Inverse followed by forward kinematics must restore the original velocity.
 *
 * This is the self-consistency check of the kinematic model: if a coefficient or
 * a sign is wrong, the round trip no longer returns the command.
 */
TEST(HeroChassisKinematics, InverseThenForwardRestoresVelocity)
{
  const ChassisGeometry geometry;

  const double test_cases[][3] = {
    { 0.5, 0.0, 0.0 },    // forward
    { 0.0, 0.5, 0.0 },    // strafe left
    { 0.0, 0.0, 0.5 },    // rotate counter-clockwise
    { 0.3, -0.2, 0.4 },   // combined motion
  };

  for (const auto& test_case : test_cases)
  {
    const double cmd_vx = test_case[0];
    const double cmd_vy = test_case[1];
    const double cmd_wz = test_case[2];

    double target_speeds[WHEEL_COUNT];
    computeWheelSpeeds(geometry, cmd_vx, cmd_vy, cmd_wz, target_speeds);

    double vx_forward = 0.0;
    double vy_forward = 0.0;
    double wz_forward = 0.0;
    computeChassisVelocity(geometry, target_speeds, &vx_forward, &vy_forward, &wz_forward);

    EXPECT_NEAR(cmd_vx, vx_forward, TOLERANCE);
    EXPECT_NEAR(cmd_vy, vy_forward, TOLERANCE);
    EXPECT_NEAR(cmd_wz, wz_forward, TOLERANCE);
  }
}

/**
 * @brief A forward command makes all four wheels spin with the same speed and sign.
 */
TEST(HeroChassisKinematics, ForwardCommandSpinsAllWheelsEqually)
{
  const ChassisGeometry geometry;

  double target_speeds[WHEEL_COUNT];
  computeWheelSpeeds(geometry, 0.5, 0.0, 0.0, target_speeds);

  const double expected = 0.5 / geometry.wheel_radius_;
  EXPECT_NEAR(expected, target_speeds[LEFT_FRONT], TOLERANCE);
  EXPECT_NEAR(expected, target_speeds[RIGHT_FRONT], TOLERANCE);
  EXPECT_NEAR(expected, target_speeds[LEFT_BACK], TOLERANCE);
  EXPECT_NEAR(expected, target_speeds[RIGHT_BACK], TOLERANCE);
}

/**
 * @brief A rotation command spins the left wheels negatively and the right wheels positively.
 */
TEST(HeroChassisKinematics, RotationSpinsLeftAndRightOppositely)
{
  const ChassisGeometry geometry;

  double target_speeds[WHEEL_COUNT];
  computeWheelSpeeds(geometry, 0.0, 0.0, 0.5, target_speeds);

  EXPECT_LT(target_speeds[LEFT_FRONT], 0.0);
  EXPECT_GT(target_speeds[RIGHT_FRONT], 0.0);
  EXPECT_LT(target_speeds[LEFT_BACK], 0.0);
  EXPECT_GT(target_speeds[RIGHT_BACK], 0.0);
}

/**
 * @brief A strafe command makes the diagonal wheels spin with the same speed.
 */
TEST(HeroChassisKinematics, StrafeSpinsDiagonalsEqually)
{
  const ChassisGeometry geometry;

  double target_speeds[WHEEL_COUNT];
  computeWheelSpeeds(geometry, 0.0, 0.5, 0.0, target_speeds);

  EXPECT_NEAR(target_speeds[LEFT_FRONT], target_speeds[RIGHT_BACK], TOLERANCE);
  EXPECT_NEAR(target_speeds[RIGHT_FRONT], target_speeds[LEFT_BACK], TOLERANCE);
}

/**
 * @brief The scaling coefficient of the yaw term must be the half wheel base plus the half wheel track.
 *
 * Using the sum of both wheel distances (0.8 m) instead of 0.4 m would halve the
 * wheel speeds needed for a given rotation command.
 */
TEST(HeroChassisKinematics, RotationUsesHalfTrackSum)
{
  const ChassisGeometry geometry;

  const double cmd_wz = 0.5;
  const double L_sum = geometry.track_width_x_ / 2 + geometry.track_width_y_ / 2;
  const double expected = L_sum * cmd_wz / geometry.wheel_radius_;

  double target_speeds[WHEEL_COUNT];
  computeWheelSpeeds(geometry, 0.0, 0.0, cmd_wz, target_speeds);

  EXPECT_NEAR(expected, std::fabs(target_speeds[RIGHT_FRONT]), TOLERANCE);

  // 误用整段轮距时，转速会翻倍 —— 这里确认当前实现没有犯这个错
  const double wrong_L_sum = geometry.track_width_x_ + geometry.track_width_y_;
  const double wrong = wrong_L_sum * cmd_wz / geometry.wheel_radius_;
  EXPECT_GT(std::fabs(wrong - std::fabs(target_speeds[RIGHT_FRONT])), 1e-6);
}

}  // namespace hero_chassis_controller

/**
 * @brief Entry point of the gtest runner.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return 0 if all tests passed, non zero otherwise.
 */
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
