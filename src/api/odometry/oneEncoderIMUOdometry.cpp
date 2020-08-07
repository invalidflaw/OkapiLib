/*
 * @author Ryan Thomas, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/oneEncoderIMUOdometry.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
OneEncoderIMUOdometry::OneEncoderIMUOdometry(const TimeUtil &itimeUtil,
                                       const std::shared_ptr<ReadOnlyChassisModel> &imodel,
                                       const ChassisScales &ichassisScales,
                                       const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    rate(itimeUtil.getRate()),
    timer(itimeUtil.getTimer()),
    model(imodel),
    chassisScales(ichassisScales) {
}

void OneEncoderIMUOdometry::setScales(const ChassisScales &ichassisScales) {
  chassisScales = ichassisScales;
}

void OneEncoderIMUOdometry::step() {
  const auto deltaT = timer->getDt();

  if (deltaT.getValue() != 0) { //
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    const auto newState = odomMathStep(tickDiff, deltaT);

    state.x += newState.x;
    state.y += newState.y;
    state.theta += newState.theta;
  }
}

OdomState OneEncoderIMUOdometry::odomMathStep(const std::valarray<std::int32_t> &itickDiff, const QTime &) {
  //TODO: Implement code for odometry step.

  if (itickDiff.size() < 2) {
    LOG_ERROR_S("OneEncoderIMUOdometry: itickDiff did not have at least two elements.");
    return OdomState{};
  }

  for (auto &&elem : itickDiff) {
    if (std::abs(elem) > maximumTickDiff) {
      LOG_ERROR("OneEncoderIMUOdometry: A tick diff (" + std::to_string(elem) +
                ") was greater than the maximum allowable diff (" +
                std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
      return OdomState{};
    }
  }

  const double currentAngle = model->getSensorVals()[1];
  const double deltaTranslation = itickDiff[0];
  const double deltaTheta = itickDiff[1];

  double moveDistance = deltaTranslation / chassisScales.straight; //Not sure what this value should be
  double dX = (moveDistance * std::cos(currentAngle * 0.0174533)); //Not sure what this number is
  double dY = (moveDistance * std::sin(currentAngle * 0.0174533));

  if (isnan(dX)) {
    dX = 0;
  }

  if (isnan(dY)) {
    dY = 0;
  }

  return OdomState{dX * meter, dY * meter, deltaTheta * radian};
}

OdomState OneEncoderIMUOdometry::getState(const StateMode &imode) const {
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    return state;
  } else {
    return OdomState{state.y, state.x, state.theta};
  }
}

void OneEncoderIMUOdometry::setState(const OdomState &istate, const StateMode &imode) {
  LOG_DEBUG("State set to: " + istate.str());
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    state = istate;
  } else {
    state = OdomState{istate.y, istate.x, istate.theta};
  }
}

std::shared_ptr<ReadOnlyChassisModel> OneEncoderIMUOdometry::getModel() {
  return model;
}

ChassisScales OneEncoderIMUOdometry::getScales() {
  return chassisScales;
}
} // namespace okapi
