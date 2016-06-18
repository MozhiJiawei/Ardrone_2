/*
 * main.cpp
 *
 *  Created on: June 18, 2016
 *      Author: ljw
 */
#include "PID.h"
#define CLIP3(_n1, _n, _n2)                                                    \
  {                                                                            \
    if (_n < _n1)                                                              \
      _n = _n1;                                                                \
    if (_n > _n2)                                                              \
      _n = _n2;                                                                \
  }

void PID::PIDXY(double error, double v_max, bool is_X) {
  double targetv, control;
  double kp = 2.0;
  if (is_X) {
    targetv = (-2 * error - lasterrorx_) * kp;
    lasterrorx_ = error;
  }
  else {
    targetv = (-2 * error - lasterrory_) * kp;
    lasterrory_ = error;
  }

  if (error > 80 || error < -80) {
    targetv += (-error + 80) * kp;
  }
  CLIP3(-v_max, targetv, v_max);

  if (is_X) {
    control = pid_vx_.getOutput(targetv - thread_.navdata.vx, 0.5);
  }
  else {
    control = pid_vy_.getOutput(targetv - thread_.navdata.vy, 0.5);
  }
  control /= 15000;
  return control;
}


double PID::PIDZ(double altitude, double tolerance) {
  double upd;
  double kp = 0.002;
  if (thread_.navdata.altd < altitude) {
    upd = kp * (altitude - thread_.navdata.altd);
  }
  else if (thread_.navdata.altd > altitude + tolerance) {
    upd = kp * (altitude + tolerance - thread_.navdata.altd);
  }
  else {
    upd = 0;
  }
  return upd;
}

void PID::PIDReset() {
  lasterrorx_ = 0;
  lasterrory_ = 0;
}
