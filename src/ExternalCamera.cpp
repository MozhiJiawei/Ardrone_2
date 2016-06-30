/*
 * DroneThread.h
 *
 *  Created on: June 21, 2016
 *      Author: ljw
 */
#include "ExternalCamera.h"
#include "ros/ros.h"
using namespace cv;

ExternalCamera::ExternalCamera(double offset_y) {
  running_ = false;
  toQuit_ = false;
  threadID_ = 0;
  pthread_mutex_init(&mutex_, 0);
  robotexist = false;
  showing_stuff_ = 0;
  offset_y_ = offset_y;
  setHomographyFromXML();
  Start();
}
bool ExternalCamera::isRobotExists() {
  return robotexist;
}

void ExternalCamera::getRobotPosition(double & robot_x, double & robot_y) {
  if (position_buffer_.empty()) {
    robot_x = 0;
    robot_y = 0;
    return; 
  }
  robot_x = position_buffer_.back().x_ - offset_y_;
  robot_y = position_buffer_.back().y_;
}

void ExternalCamera::setHomographyFromXML() {
  cv::FileStorage fs("homography.xml", FileStorage::READ);
  fs["Homography_me"] >> homography_me_;
  fs["Homography_enemy"] >> homography_enemy_;
}

void ExternalCamera::FindHomography() {
  pthread_mutex_lock(&mutex_);
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
  img = camera_img_.clone();
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
  pthread_mutex_unlock(&mutex_); 
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

bool ExternalCamera::isRobotForward() {
  if (position_buffer_.size() >= 5) {
    double distance, distance_x, distance_y;
    distance_x = position_buffer_.back().x_ - position_buffer_.front().x_;
    distance_y = position_buffer_.back().y_ - position_buffer_.front().y_;
    distance = distance_x * distance_x + distance_y * distance_y;
    std::cout << distance << std::endl;
    if (position_buffer_.back().x_ < position_buffer_.front().x_) {
      return false;
    }
  }
  return true;
}

void ExternalCamera::ChangeShowing() {
  if(++showing_stuff_ ==3){
    showing_stuff_ = 0;
  }
}

void ExternalCamera::ImageProcess() {
  IplImage img = IplImage(img_me_), *sourImg = &img, *Imgthresh;
  CvSize imgSize = {800, 700};
  Imgthresh = cvCreateImage(imgSize ,8 ,1);
  int bl = 0, gr = 0, re = 0;
  unsigned char *p = (unsigned char *)sourImg->imageData;
  unsigned char *q = (unsigned char *)Imgthresh->imageData;
  for (int i = 0; i < sourImg->width; ++i) {
    for (int j = 0; j < sourImg->height; ++j) {
      bl = (int)p[i*sourImg->nChannels + j*sourImg->widthStep];
      gr = (int)p[i*sourImg->nChannels + j*sourImg->widthStep+1];
      re = (int)p[i*sourImg->nChannels + j*sourImg->widthStep+2];
      if ((re - bl) > 50 && (re - gr)>50)
        q[i*Imgthresh->nChannels + j*Imgthresh->widthStep] = 255;
      else
        q[i*Imgthresh->nChannels + j*Imgthresh->widthStep] = 0;
    }
  }

  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *contour = 0, *robcont = 0, *tempcont = 0;
  int numofcont = cvFindContours(Imgthresh, storage, &contour,
      sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  if (numofcont > 0) {
    tempcont = contour;
    robcont = contour;
    CvPoint2D32f robcenter, tempcenter;
    float robR = 0, tempR = 0;

    while (tempcont != NULL) {
      cvMinEnclosingCircle(tempcont, &tempcenter, &tempR);
      if (tempR > robR) {
        robcenter = tempcenter;
        robR = tempR;
        robcont = tempcont;
      }
      tempcont = tempcont->h_next;
    }
    if (robR > 20) {
      robotexist = true;
      RobotPosition rob_pos((400 - robcenter.y)*1.8 / 300, 
          (400 - robcenter.x)*1.8 / 300);//calculate the real position

      position_buffer_.push_back(rob_pos);
      if (position_buffer_.size() > 10) {
        position_buffer_.pop_front();
      }
    }
    else {
      robotexist = false;
      if(!position_buffer_.empty()) {
        position_buffer_.pop_front();
      }
    }
  }
  else {
    robotexist = false;
    if(!position_buffer_.empty()) {
      position_buffer_.pop_front();
    }
  }
  cvReleaseMemStorage(&storage);
  cvReleaseImage(&Imgthresh);
}

void ExternalCamera::InitDstPoints(int rows, int columns) {
  double scale = 100;
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < columns; ++x) {
      dst_points_.push_back(cv::Point2f((x+2) * scale, (y+2) * scale));
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
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    std::cout << "Cannot Open Video." << std::endl;
    return;
  }
  if (homography_me_.cols != 3 || homography_enemy_.cols != 3) {
    cap >> camera_img_;
    FindHomography();
  }
  ros::Rate rate(5);
  while (ros::ok() && !toQuit_) {
    pthread_mutex_lock(&mutex_);
    cap >> camera_img_;
    cv::warpPerspective(camera_img_, img_me_, homography_me_, 
        cv::Size(800, 700));

    cv::warpPerspective(camera_img_, img_enemy_, homography_enemy_,
        cv::Size(800, 700));

    pthread_mutex_unlock(&mutex_);
    ImageProcess();
    rate.sleep();
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
