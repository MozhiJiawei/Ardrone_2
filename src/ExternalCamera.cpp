/*
 * DroneThread.h
 *
 *  Created on: June 21, 2016
 *      Author: ljw
 */
#include "ExternalCamera.h"

ExternalCamera::ExternalCamera() {
  running_ = false;
  toQuit_ = false;
  threadID_ = 0;
  pthread_mutex_init(&mutex_, 0);
  Start();
}
bool ExternalCamera::isRobotExists() {
  return false;
}

void ExternalCamera::getRobotPosition(int & robot_x, int & robot_y) {
}

void ExternalCamera::setHomographyFromXML() {
}

void ExternalCamera::FindHomography() {
}

void ExternalCamera::ImageProcess() {
}

void ExternalCamera::Start() {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&threadID_, &attr, ThreadProc, this);
}

void ExternalCamera::Loop() {
  cv::VideoCapture cap(1);
  cv::namedWindow("Video", 1);
  cv::Mat frame;
  while (!toQuit_) {
    cap >> frame;
    cv::imshow("Video", frame);
    char c = cv::waitKey(3);
  }

}

void ExternalCamera::End() {
  if (running_) {
    toQuit_ = true;
    std::cout << "waiting the External Camera thread to quit..." 
      << std::endl;

    pthread_mutex_lock(&mutex_);
    pthread_mutex_unlock(&mutex_);
    pthread_join(threadID_, 0);
    std::cout << "External Camera thread quit!" << std::endl;
  }
}

void * ExternalCamera::ThreadProc(void * data) {
  ExternalCamera* external_camera = (ExternalCamera*)data;
  external_camera->Loop();
}
