#include "ExternalCamera.h"

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
  pthread_create(&_threadID, &attr, ThreadProc, this);
}

void ExternalCamera::Loop() {
  cv::VideoCapture cap(1);
  cv::namedWindow("Video", 1);
  CV_ASSERT(cap.isOpened);
  cv::Mat frame;
  while (!toQuit_) {
    cap >> frame;
    cv::imshow("Video", frame);
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
