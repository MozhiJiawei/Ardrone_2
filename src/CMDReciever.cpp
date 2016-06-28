/*
 * CMDReciever.cpp
 *
 *  Created on: May 9, 2016
 *      Author: Ljw
 */

#include "CMDReciever.h"
#include "time.h"


CMDReciever::CMDReciever(const char* fileName, ARDrone& drone) :_drone(drone) {
  _filePath = fileName;
  _mode = STOP;
  pthread_mutex_init(&_mode_mutex, 0);
  pthread_mutex_init(&_time_mutex, 0);
}

CMDReciever::~CMDReciever() {
  pthread_mutex_lock(&_mode_mutex);
  pthread_mutex_lock(&_time_mutex);
}

void CMDReciever::SaveImage(const cv::Mat& src) {
  char filename[50];
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  if(src.rows > 360) {
    sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.bmp",
        a->tm_mday, a->tm_hour, a->tm_min, a->tm_sec + 1);

  }
  else {
    sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.bmp",
        a->tm_mday, a->tm_hour, a->tm_min, a->tm_sec);

  }

  cout << "Save image to" << endl << filename << endl;
  cv::imwrite(filename, src);
}

ModeType CMDReciever::GetMode() {
  pthread_mutex_lock(&_mode_mutex);
  ModeType cur_mode = _mode;
  pthread_mutex_unlock(&_mode_mutex);
  /*switch (_mode) {
  case TAKEOFF:
    cout << "TAKEOFF" << endl;
    break;
  case STOP:
    cout << "STOP" << endl;
    break;
  case START:
    cout << "START" << endl;
    break;
  default:
    break;
  }*/
  return cur_mode;
}

void CMDReciever::SetMode(ModeType mode) {
  pthread_mutex_lock(&_mode_mutex);
  _mode = mode;
  pthread_mutex_unlock(&_mode_mutex);

  if (_mode == START) {
    _is_reset = true;
  }
}

void CMDReciever::RunNextMode(ModeType mode, double y_left, double x_forward,
  double z_up, double angle_turn) {

  if (_mode != mode) {
    pthread_mutex_lock(&_mode_mutex);
    _mode = mode;
    pthread_mutex_unlock(&_mode_mutex);
    return;
  }

  switch (mode) {
  case WAITING:
  case TOCENTER:
  case TOROBOT:
  case FOLLOWROBOT:
  case SEARCHING:
    _drone.move(y_left, x_forward, z_up, angle_turn);
    break;
  default:
    break;
  }
}

clock_t CMDReciever::GetManualTime() {
  pthread_mutex_lock(&_mode_mutex);
  clock_t manul_time = _manul_time;
  pthread_mutex_unlock(&_mode_mutex);

  return manul_time;
}

void CMDReciever::SetManualTime() {
  pthread_mutex_lock(&_mode_mutex);
  _manul_time = clock();
  pthread_mutex_unlock(&_mode_mutex);
}

void CMDReciever::Key_X_LAND() {
  _drone.land();
  SetManualTime();
}

void CMDReciever::Key_Z_Takeoff() {
  _drone.takeOff();
  SetManualTime();
}

void CMDReciever::Key_H_Hover() {
  _drone.hover();
  SetManualTime();
}
void CMDReciever::Key_I_Up() {
  _drone.moveUp((float) 0.2);
  SetManualTime();
}
void CMDReciever::Key_K_Down() {
  _drone.moveDown((float) 0.2);
  SetManualTime();
}
void CMDReciever::Key_J_Turnleft() {
  _drone.turnLeft((float) 0.4);
  SetManualTime();
}
void CMDReciever::Key_L_Turnright() {
  _drone.turnRight((float) 0.4);
  SetManualTime();
}
void CMDReciever::Key_W_Forward() {
  _drone.moveForward((float) 0.2);
  SetManualTime();
}
void CMDReciever::Key_S_Backward() {
  _drone.moveBackward((float) 0.2);
  SetManualTime();
}
void CMDReciever::Key_A_Left() {
  _drone.moveLeft((float) 0.2);
  SetManualTime();
}
void CMDReciever::Key_D_Right() {
  _drone.moveRight((float) 0.2);
  SetManualTime();
}
