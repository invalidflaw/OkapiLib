/*
 * @author Ryan Thomas, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/oneEncoderIMUSkidSteerModel.hpp"


//TODO: Modify for OneEncoderIMUSkidSteerModel



namespace okapi {
OneEncoderIMUSkidSteerModel::OneEncoderIMUSkidSteerModel(
    std::shared_ptr<AbstractMotor> ileftSideMotor,
    std::shared_ptr<AbstractMotor> irightSideMotor,
    std::shared_ptr<ContinuousRotarySensor> ileftEnc,
    std::shared_ptr<ContinuousRotarySensor> irightEnc,
    std::shared_ptr<ContinuousRotarySensor> iforwardEnc,
    std::shared_ptr<ContinuousRotarySensor> irotationSensor,
    const double imaxVelocity,
    const double imaxVoltage)

    : SkidSteerModel(std::move(ileftSideMotor),
                     std::move(irightSideMotor),
                     std::move(ileftEnc),
                     std::move(irightEnc),
                     imaxVelocity,
                     imaxVoltage),
      forwardSensor(std::move(iforwardEnc)),
      rotationSensor(std::move(irotationSensor)) {
  }

std::valarray<std::int32_t> OneEncoderIMUSkidSteerModel::getSensorVals() const {
  //get senor values of encoder and imu
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(forwardSensor->get()),
                                     static_cast<std::int32_t>(rotationSensor->get())};
}

void OneEncoderIMUSkidSteerModel::resetSensors() {
  rotationSensor.reset();
  forwardSensor.reset();
}
} // namespace okapi
