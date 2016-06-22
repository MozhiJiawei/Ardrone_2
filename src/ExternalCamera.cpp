/*
 * DroneThread.h
 *
 *  Created on: June 21, 2016
 *      Author: ljw
 */
#include "ExternalCamera.h"
using namespace cv;

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
  if (cur_img_.empty()) {
    cout << "find homography failed" << std::endl;
    cout << "cannot open external camera!!" << std::endl;
    return;
  }
  DataCallback data_cb;
  cv::Mat img;
  cv::namedWindow("Find Homo", WINDOW_AUTOSIZE);
  pthred_mutex_lock(&mutex_);
  img = cur_img_.clone();
  pthread_mutex_unlock(&mutex_); 
  data_cb.input_img = img;
  cv::imshow("Find Homo", img);
  cv::setMouseCallback("Find Homo", onMouse, &data_cb);
  while (true) {
    int c = cv::waitKey(0);
    if ((char)c == 27) {
      cv::destroyWindow("Find Homo");
      break;
    }
  }
  cv::findHomography(src_points, dst_points_, homography_);
  cv::FileStorage fs("homography.xml", FileStorage::WRITE);
  fs >> "Homography" >> homography_;
}

void ExternalCamera::ImageProcess() {
}

void ExternalCamera::InitDstPoints(int rows, int columns) {

}

void ExternalCamera::onMouse(int events, int x, int y, int flag, void * data) {
  DataCallback* data_cb = (DataCallback *)data;
  cv::Mat img = data_cb->input_img.clone();
  if (events == EVENT_LBUTTONDOWN) {
    data_cb->src_point_.push_back(cv::Point2f(x, y));
  }
  else if(events == EVENT_RBUTTONDOWN){
    data_cb->src_point_.pop_back();
  }
  for (int i = 0; i < data_cb->src_point_.size(); ++i) {
    cv::circle(img, 
      cv::Point(data_cb->src_point_[i].x, data_cb->src_point_[i].y),
      3, cv::Scalar(0));
  }
  cv::imshow("Find Homo", img);
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
    cur_img_ = frame;
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
