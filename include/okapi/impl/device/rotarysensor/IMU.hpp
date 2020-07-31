/*
 * @author Alex Riensche, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/impl/device/motor/motor.hpp"

namespace okapi {

enum class IMUAxes{
  x = 0, ///< X axis
  y = 1, ///< Y axis
  z = 2  ///< Z axis
};

class IMU : public ContinuousRotarySensor{
  public:
  /**
   * An inertial sensor on the given port. If the port has not previously been configured as an IMU,
   * then the constructor will block for 2 seconds for calibration. The IMU returns double angle results
   *
   * @param iport: The port to use the inertial sensor from
   * @param iaxis: The axis of the inertial sensor to measure
   */
   IMU(std::uint8_t iport, IMUAxes iaxis);

   virtual ~IMU();

   /**
    * Get the current number of degrees the sensor has spun about the specified axis
    *
    * This value is unbounded. Clockwise represents positive values and counterclockwise represents negative
    * @return the current sensor rotation value, or a value of pros 'errno'
    * ENXIO
    * ENODEV
    * EAGAIN
    */
    double get() const override;

    double getRemapped(double iupperBound = 180, double ilowerBound = -180);

    std::int32_t reset() override;

    double controllerGet() override;

    protected:
    pros::Imu gyro;


};
} // namespace okapi
