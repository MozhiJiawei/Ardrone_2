/*
 * DroneThread.h
 *
 *  Created on: June 21, 2016
 *      Author: ljw
 */

#ifndef EXTERNALCAMERA_H__
#define EXTERNALCAMERA_H__

#include "pthread.h"
#include "opencv2/opencv.hpp"
#include <list>

class ExternalCamera {
public:
  ExternalCamera();
  ~ExternalCamera() {
    End();
  }

  bool isRobotExists();
  void getRobotPosition(int &robot_x, int &robot_y);
  void setHomographyFromXML();
  void FindHomography();

private:
  struct RobotPosition {
  public:
    RobotPosition(int x, int y): x_(x), y_(y) {}
    int x_;
    int y_
  };
  std::list<RobotPosition> position_buffer_;
  cv::Mat cur_img_;
  cv::Mat homography_;
  
  void ImageProcess();

  bool running_;
  bool toQuit_;
  pthread_t threadID_;
  pthread_mutex_t mutex_;
  void Start();
  void Loop();
  void End();
  static void * ThreadProc(void* data);
};

#endif /*EXTERNALCAMERA_H__*/
