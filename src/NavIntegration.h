/*
* ArdroneTf.h
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#ifndef NAVINTEGRATION_H__
#define NAVINTEGRATION_H__

#include "pthread.h"

class NavIntegration {
public:
  NavIntegration();
  ~NavIntegration();

  void Clear();
  void Add(double vx, double vy, double duration);
  void Get(double &x, double &y);
private:
  double x_;
  double y_;
  pthread_mutex_t mutex_;
};

#endif /*NAVINTEGRATION_H__*/