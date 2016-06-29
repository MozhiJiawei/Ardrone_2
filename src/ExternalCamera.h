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
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <list>

static void onMouse(int events, int x, int y, int flag, void *data);

struct DataCallback {
  std::vector<cv::Point2f> src_point_;
  cv::Mat input_img;
};

class ExternalCamera {
public:
  ExternalCamera(double offset_y);
  ~ExternalCamera() {
    End();
  }

  bool isRobotExists();
  void getRobotPosition(double &robot_x, double &robot_y);
  void setHomographyFromXML();
  void FindHomography();
  bool getCurImage(cv::Mat &img);
  bool isRobotForward();
  void ChangeShowing();

private:
  struct RobotPosition {
  public:
    RobotPosition(double x, double y): x_(x), y_(y) {}
    double x_;
    double y_;
  };
  std::list<RobotPosition> position_buffer_;
  std::vector<cv::Point2f> dst_points_;
  cv::Mat camera_img_;
  cv::Mat img_me_;
  cv::Mat img_enemy_;
  cv::Mat homography_me_;
  cv::Mat homography_enemy_;
  // 0 -- camera(default) 
  // 1 -- me
  // 2 -- enemy
  int showing_stuff_;
  
  void ImageProcess();
  void InitDstPoints(int rows, int columns);

  double offset_y_;
  bool robotexist;
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
