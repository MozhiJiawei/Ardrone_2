/*
* ArdroneTf.h
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#include "NavIntegration.h"

NavIntegration::NavIntegration() :x_(0), y_(0) {
  pthread_mutex_init(&mutex_, 0);
}

NavIntegration::~NavIntegration() {
  pthread_mutex_lock(&mutex_);
  pthread_mutex_unlock(&mutex_);
}

void NavIntegration::Clear() {
  pthread_mutex_lock(&mutex_);
  x_ = 0;
  y_ = 0;
  pthread_mutex_unlock(&mutex_);
}

void NavIntegration::Add(double vx, double vy, double duration) {
  pthread_mutex_lock(&mutex_);
  x_ += vx * duration;
  y_ += vy * duration;
  pthread_mutex_unlock(&mutex_);
}

void NavIntegration::Get(double & x, double & y) {
  pthread_mutex_lock(&mutex_);
  x = x_;
  y = y_;
  pthread_mutex_unlock(&mutex_);
}
