/*
 * @author Alex Riensche, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

 // NEEDS API DOCUMENTATION UPDATES!!!!!
 
#pragma once

#include "api.h"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {

enum class IMUAxes{
  z, ///< Yaw Axis
  y, ///< Pitch Axis
  x  ///< Roll Axis
};

class IMU : public ContinuousRotarySensor{
  public:
  /**
   * An inertial sensor on the given port. If the port has not previously been configured as an IMU,
   * then the constructor will block for 2 seconds for calibration. The IMU returns double angle results
   *
   * @param iport: The port to use the inertial sensor from
   * @param iaxis: The axis of the inertial sensor to measure, default z
   */
   IMU(std::uint8_t iport,IMUAxes iaxis = IMUAxes::z);

   virtual ~IMU();

   /**
    * Get the current rotation from the iaxis
    *
    * @return the current sensor value, or ``PROS_ERR`` on a failure.
    */
    double get() const override;

    /**
     * Get the current sensor value remapped into the target range ([1800, -1800] by default).
     *
     * @param iupperBound the upper bound of the range.
     * @param ilowerBound the lower bound of the range.
     * @return the remapped sensor value.
     */
    double getRemapped(double iupperBound = 180, double ilowerBound = -180) const
      __attribute__((optimize(3)));

    /**
     * Reset the sensor to zero.
     *
     * @return `1` on success, `PROS_ERR` on fail
     */
    std::int32_t reset() override;

    /**
     * Calibrates the IMU
     *
     * @return `1` on success, `PROS_ERR` on fail
     */
    std::int32_t calibrate();


    /**
     * Get the sensor value for use in a control loop. This method might be automatically called in
     * another thread by the controller.
     *
     * @return the current sensor value, or ``PROS_ERR`` on a failure.
     */
    double controllerGet() override;

    protected:
    std::uint8_t port;
    double offset = 0; // this offset must invert based on the value currrently measured
    IMUAxes axis;
};
} // namespace okapi
