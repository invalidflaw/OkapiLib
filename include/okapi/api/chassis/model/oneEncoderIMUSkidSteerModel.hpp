/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/model/skidSteerModel.hpp"

namespace okapi {
class OneEncoderIMUSkidSteerModel : public SkidSteerModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc The left side encoder.
   * @param irightEnc The right side encoder.
   * @param iforwardEnc forward encoder (mounted parrallel to the left and right wheels)
   * @param irotationSensor imu
   */
  OneEncoderIMUSkidSteerModel(std::shared_ptr<AbstractMotor> ileftSideMotor,
                             std::shared_ptr<AbstractMotor> irightSideMotor,
                             std::shared_ptr<ContinuousRotarySensor> ileft,
                             std::shared_ptr<ContinuousRotarySensor> iright,
                             std::shared_ptr<ContinuousRotarySensor> iforwardEnc,
                             std::shared_ptr<ContinuousRotarySensor> irotationSensor,
                             double imaxVelocity,
                             double imaxVoltage);

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {forward, rotation}
   */
  std::valarray<std::int32_t> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  void resetSensors() override;

  protected:
  std::shared_ptr<ContinuousRotarySensor> forwardSensor;
  std::shared_ptr<ContinuousRotarySensor> rotationSensor;
};
} // namespace okapi
