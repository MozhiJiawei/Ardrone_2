/*
 * CMDReciever.h
 *
 *  Created on: May 9, 2016
 *      Author: Ljw
 */

#ifndef CMDRECIEVER_H_
#define CMDRECIEVER_H_

#include "pthread.h"
#include "ARDrone.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;

enum ModeType { TAKEOFF, LAND, STOP, FLYING, SEARCHING, START, MANUL };

class CMDReciever {
public:
  CMDReciever(const char* fileName, ARDrone& drone);
  virtual ~CMDReciever();

  void SaveImage(const cv::Mat& src);
  ModeType GetMode();
  clock_t GetManualTime();
  void SetMode(ModeType mode);
  void RunNextMode(ModeType mode, double y_left, double x_forward,
    double z_up, double angle_turn);

  void Key_X_LAND();
  void Key_Z_Takeoff();
  void Key_H_Hover();
  void Key_I_Up();
  void Key_K_Down();
  void Key_J_Turnleft();
  void Key_L_Turnright();
  void Key_W_Forward();
  void Key_S_Backward();
  void Key_A_Left();
  void Key_D_Right();

  bool _is_reset;
private:
  ModeType _mode;
  ARDrone& _drone;

  clock_t _manul_time;
  pthread_mutex_t _mode_mutex;
  pthread_mutex_t _time_mutex;
  const char* _filePath;
  ofstream log;

  void SetManualTime();
};

#endif /* CMDReciever_H_ */
