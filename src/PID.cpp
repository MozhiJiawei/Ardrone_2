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

double PID::PIDXY(double error, double v_max, bool is_X) {
  double targetv, control;
  double kp = 2.0;
  if (is_X) {
    targetv = (-2 * error + lasterrorx_) * kp;
    lasterrorx_ = error;
  }
  else {
    targetv = (-2 * error + lasterrory_) * kp;
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


double PID::PIDZ(double reference, double tolerance, bool is_altd) {
  double upd, control_stuff;
  double kp;
  if(is_altd) {
    control_stuff = thread_.navdata.altd; 
    kp = 0.002;
  } else {
    if(find_rob_.doesGroundCenterExist()) {
      control_stuff = find_rob_.getGroundCenterRadius();
    }
    else if(find_rob_.doesRobotExist()) {
      control_stuff = find_rob_.getRobRadius();
    }
    kp = -0.01;
  }

  if (control_stuff < reference) {
    upd = kp * (reference - control_stuff);
  } else if (control_stuff > reference + tolerance) {
    upd = kp * (reference + tolerance - control_stuff);
  } else {
    upd = 0;
  }
  return upd;
}

void PID::PIDReset() {
  lasterrorx_ = 0;
  lasterrory_ = 0;
}

