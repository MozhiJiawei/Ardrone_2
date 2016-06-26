/*
 * DroneThread.h
 *
 *  Created on: June 21, 2016
 *      Author: ljw
 */
#include "ExternalCamera.h"
#include "ros/ros.h"
using namespace cv;

ExternalCamera::ExternalCamera() {
  running_ = false;
  toQuit_ = false;
  threadID_ = 0;
  pthread_mutex_init(&mutex_, 0);
  showing_stuff_ = 0;
  setHomographyFromXML();
  Start();
}
bool ExternalCamera::isRobotExists() {
  return false;
}

void ExternalCamera::getRobotPosition(int & robot_x, int & robot_y) {
}

void ExternalCamera::setHomographyFromXML() {
  cv::FileStorage fs("homography.xml", FileStorage::READ);
  fs["Homography_me"] >> homography_me_;
  fs["Homography_enemy"] >> homography_enemy_;
}

void ExternalCamera::FindHomography() {
  bool is_enemy = false;
  cv::FileStorage fs("homography.xml", FileStorage::WRITE);
  if (camera_img_.empty()) {
    std::cout << "find homography failed" << std::endl;
    std::cout << "cannot open external camera!!" << std::endl;
    return;
  }
  InitDstPoints(5, 5);
  DataCallback data_cb;
  cv::Mat img;
  cv::namedWindow("Find Homo", WINDOW_AUTOSIZE);
  pthread_mutex_lock(&mutex_);
  img = camera_img_.clone();
  pthread_mutex_unlock(&mutex_); 
  data_cb.input_img = img;
  cv::imshow("Find Homo", img);
  cv::setMouseCallback("Find Homo", onMouse, &data_cb);
  while (true) {
    int c = cv::waitKey(0);
    if ((char)c == 27) {
      if(!is_enemy) {
        homography_me_ = cv::findHomography(data_cb.src_point_, dst_points_);
        fs << "Homography_me" << homography_me_;
        is_enemy = true;
        data_cb.src_point_.clear();
        continue;
      }
      else {
        homography_enemy_ = cv::findHomography(data_cb.src_point_,dst_points_);
        fs << "Homography_enemy" << homography_enemy_;
        cv::destroyWindow("Find Homo");
        break;
      }
    }
  }
}

bool ExternalCamera::getCurImage(cv::Mat &img) {
  pthread_mutex_lock(&mutex_);
  if (camera_img_.empty()) {
    pthread_mutex_unlock(&mutex_);
    return false;
  }
  switch(showing_stuff_) {
    case 0:
      img = camera_img_.clone();
      break;
    case 1:
      img = img_me_.clone();
      break;
    case 2:
      img = img_enemy_.clone();
      break;
  }
  pthread_mutex_unlock(&mutex_);
  return true;
}

void ExternalCamera::ChangeShowing() {
  if(++showing_stuff_ ==3){
    showing_stuff_ = 0;
  }
}

void ExternalCamera::ImageProcess() {
}

void ExternalCamera::InitDstPoints(int rows, int columns) {
  double scale = 100;
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < columns; ++x) {
      dst_points_.push_back(cv::Point2f((x+1) * scale, (y+1) * scale));
    }
  }
}

static void onMouse(int events, int x, int y, int flag, void * data) {
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
      4, cv::Scalar(0), 3);
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
  if(!cap.isOpened()) {
    std::cout << "Cannot Open Video." << std::endl;
    return;
  }
  //cv::namedWindow("Video", 1);
  if (homography_me_.cols != 3 || homography_enemy_.cols != 3) {
    cap >> camera_img_;
    FindHomography();
  }
  ros::Rate rate(20);
  while (ros::ok() && !toQuit_) {
    pthread_mutex_lock(&mutex_);
    cap >> camera_img_;
    cv::warpPerspective(camera_img_, img_me_, homography_me_, 
        cv::Size(600, 600));

    cv::warpPerspective(camera_img_, img_enemy_, homography_enemy_,
        cv::Size(600, 600));

    pthread_mutex_unlock(&mutex_);
    rate.sleep();
    //switch(showing_stuff_) {
    //  case 0:
    //    cv::imshow("Video", camera_img_);
    //    break;
    //  case 1:
    //    cv::imshow("Video", img_me_);
    //    break;
    //  case 2:
    //    cv::imshow("Video", img_enemy_);
    //    break;
    //}
    //cv::waitKey(5);
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
