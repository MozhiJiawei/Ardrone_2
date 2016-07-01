
/*
 * main.cpp
 *
 *  Created on: June 18, 2016
 *      Author: ljw
 */
#ifndef PID_H__
#define PID_H__
#include "PIDController.h"
#include "ROSThread.h"
#include "FindRob.h"

class PID {
public:
  PID(ROSThread &thread, FindRob &find_rob): thread_(thread), 
      find_rob_(find_rob), lasterrorx_(0), lasterrory_(0) {
    
    double vkp = 5, vkd = 20, vki = 0;
    pid_vx_.setParam(vkp, vki, vkd, 2);
    pid_vy_.setParam(vkp, vki, vkd, 2);
  }

  ~PID() {}
  double PIDXY(double error, double v_max, bool is_X = true);
  double PIDZ(double altitude, double tolerance, bool is_robot = true);
  void PIDReset();
private:
  ROSThread &thread_;
  FindRob &find_rob_;
  PIDController pid_vx_;
  PIDController pid_vy_;
  double lasterrorx_;
  double lasterrory_;
};
#endif /*PID_H__*/
