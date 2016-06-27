/*
 * NavdataThread.cpp
 *
 *  Created on: May 15, 2013
 *      Author: tsou
 */

#include "ROSThread.h"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <iostream>
#include <iomanip>
using namespace std;

//compute the delay between video and imu, and substract the delay 
static double getVideoTimeByIMUTime(uint32_t sec, uint32_t usec, double imutime) {
  static int num = 0;
  static double s_ds = 0;
  static bool bCal = true;

  double tmV = (sec + usec / 1000000000.0);
  if (num < 20) {
    s_ds += tmV - imutime;
    num++;
  }
  else {
    if (bCal) {
      s_ds /= num;
      bCal = false;
    }
    else {
      return tmV - s_ds;
    }
  }
  return imutime;
}

ROSThread::ROSThread(IMURecorder& imu, VideoRecorder& vid, CMDReciever& cmd, 
    ExternalCamera& ex_cam, NavIntegration& drone_NI) : imuRec(imu), vidRec(vid),
    cmdRec(cmd), ex_cam_(ex_cam), drone_NI_(drone_NI) {

  running = false;
  toQuit = false;
  recording = false;
  showVideo = true;
  threadId = 0;
  cbROSThread = 0;
  lasttmIMU = 0;
  start();
}

ROSThread::~ROSThread() {
  end();
}

void* ROSThread::threadProc(void* data) {
  ROSThread* thread = (ROSThread*)data;
  thread->loop();
  return 0;
}

void ROSThread::imuCb(const sensor_msgs::Imu::ConstPtr imuPtr) {
  IMUData imu;
  //imu.tm = tmIMU;
  imu.tm = imuPtr->header.stamp.toSec();
  imu.a[0] = imuPtr->linear_acceleration.x;
  imu.a[1] = imuPtr->linear_acceleration.y;
  imu.a[2] = imuPtr->linear_acceleration.z;

  imu.g[0] = imuPtr->angular_velocity.x;
  imu.g[1] = imuPtr->angular_velocity.y;
  imu.g[2] = imuPtr->angular_velocity.z;

  imuRec.addBack(imu);

  if (cbROSThread) {
    (*cbROSThread)();
  }
}

void ROSThread::cmdCb(const keyboard::Key::ConstPtr msg) {
  double tm;
  switch (msg->code) {
  case (keyboard::Key::KEY_g) :
    cmdRec.SetMode(START);
    break;
  case keyboard::Key::KEY_x:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_X_LAND();
    break;
  case keyboard::Key::KEY_z:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_Z_Takeoff();
    break;
  case keyboard::Key::KEY_h:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_H_Hover();
    break;
  case keyboard::Key::KEY_i:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_I_Up();
    break;
  case keyboard::Key::KEY_k:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_K_Down();
    break;
  case keyboard::Key::KEY_j:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_J_Turnleft();
    break;
  case keyboard::Key::KEY_l:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_L_Turnright();
    break;
  case keyboard::Key::KEY_w:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_W_Forward();
    break;
  case keyboard::Key::KEY_s:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_S_Backward();
    break;
  case keyboard::Key::KEY_a:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_A_Left();
    break;
  case keyboard::Key::KEY_d:
    cmdRec.SetMode(MANUL);
    cmdRec.Key_D_Right();
    break;
  case keyboard::Key::KEY_r:
    ex_cam_.FindHomography();
    break;
  case keyboard::Key::KEY_t:
    ex_cam_.ChangeShowing();
    break;
  default:
    cv::Mat curImg;
    vidRec.getImage(curImg, tm);
    if (!curImg.empty()) {
      cout << "save!" << endl;
      cmdRec.SaveImage(curImg);
    }
    else {
      cout << "no image!" << endl;
    }
    if (ex_cam_.getCurImage(curImg)) {
      cout << "save!" << endl;
      cmdRec.SaveImage(curImg);
    }
    else {
      cout << "no ex image" << endl;
    }
    break;
  }
  cout << msg->code << endl;
  cout << keyboard::Key::KEY_g << endl;
}

void ROSThread::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navPtr) {
  tmIMU = navPtr->tm / 1000000.0;
  navdata = *navPtr;
  if (lasttmIMU != 0) {
    drone_NI_.Add(navdata.vx, navdata.vy, tmIMU - lasttmIMU);
  }
  lasttmIMU = tmIMU;
}

void ROSThread::vidCb(const sensor_msgs::ImageConstPtr img) {
  // convert to CVImage
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img,
    sensor_msgs::image_encodings::BGR8);

  //	double tmVid = getVideoTimeByIMUTime(img->header.stamp.sec, img->header.stamp.nsec,
  //			tmIMU);

  double tmVid = img->header.stamp.sec + img->header.stamp.nsec*1e-9;

  vidRec.addBack(tmVid, cv_ptr->image);
  // vidRec.newframe=true;
}

void ROSThread::odoCb(const nav_msgs::Odometry::ConstPtr odoPtr) {
  _odometry = *odoPtr;
}

void ROSThread::start() {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&threadId, &attr, threadProc, this);
}

void ROSThread::end() {
  if (running) {
    toQuit = true;
    ROS_INFO("waiting the drone thread to quit...\n");
    pthread_join(threadId, 0);
    ROS_INFO("Drone thread quits!\n");
  }
}

void ROSThread::loop() {
  running = true;
  cout << "starting navdata thread...\n" << endl;

  navsub = node.subscribe(node.resolveName("ardrone/navdata"), 1,
    &ROSThread::navdataCb, this);

  imusub = node.subscribe(node.resolveName("ardrone/imu"), 1,
    &ROSThread::imuCb, this);

  cmdsub = node.subscribe(node.resolveName("keyboard/keydown"), 1,
    &ROSThread::cmdCb, this);

  odosub = node.subscribe(node.resolveName("ardrone/odometry"), 1,
    &ROSThread::odoCb, this);

  if (showVideo) {
    vidsub = node.subscribe(node.resolveName("ardrone/image_raw"), 1,
      &ROSThread::vidCb, this);
  }

  while (!toQuit && node.ok()) {
    ros::spinOnce();
  }
  cout << "Drone thread exits!" << endl;
  running = false;
}
