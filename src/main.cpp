
/*
 * main.cpp
 *
 *  Created on: May 14, 2015
 *      Author: ljw
 */

#include <iostream>
#include <iomanip>
#include "ros/ros.h"
#include "math/SL_Matrix.h"
#include "fstream"
#include "std_msgs/String.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageIO.h"

#include "tools/GUI_ImageViewer.h"
#include "tools/SL_Print.h"
#include "tools/SL_DrawCorners.h"

#include "AffineTransform.h"
#include "ARDrone.h"
#include "ArdroneTf.h"
#include "CMDReciever.h"
#include "GridDetector.h"
#include "IMURecorder.h"
#include "FindRob.h"
#include "PIDController.h"
#include "PredictNumber.h"
#include "ROSThread.h"
#include "VideoRecorder.h"
#include "SearchNumber.h"

#include "time.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "opencv2/legacy/blobtrack.hpp"
#include "math.h"

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
using namespace std;

static int mGrids = 5;
static int nGrids = 6;
static int setcamera = 0;

void writeFeatPts(const vector<cv::Point2f> &featPts, const char *filePath) {
  ofstream file(filePath);
  if (!file)
    repErr("cannot open '%s' to write!", filePath);

  for (size_t i = 0; i < featPts.size(); i++) {
    file << featPts[i].x << " " << featPts[i].y << endl;
  }
}

void writeIMUMeas(const vector<IMUData> &imumeas, const char *filePath) {
  std::ofstream file(filePath);
  if (!file)
    repErr("cannot open '%' to write!", filePath);

  for (size_t i = 0; i < imumeas.size(); i++) {
    imumeas[i].write(file);
  }
  file.close();
}

void LogCurTime(ofstream &log) {
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  log << endl
      << a->tm_mday << " " << a->tm_hour << ":" << a->tm_min << ":" << a->tm_sec
      << endl;
}

#define CLIP3(_n1, _n, _n2)                                                    \
  {                                                                            \
    if (_n < _n1)                                                              \
      _n = _n1;                                                                \
    if (_n > _n2)                                                              \
      _n = _n2;                                                                \
  }
ofstream fout("/home/mozhi/Record/test.txt", ios::app);

void *Control_loop(void *param) {
  ARDrone drone;
  drone.setup();
  ArdroneTf drone_tf("/home/mozhi/Logs/tf.txt");
  CMDReciever cmdreader("/home/mozhi/Logs/cmd.txt", drone);
  IMURecorder imureader("/home/mozhi/Record/imu.txt");
  VideoRecorder videoreader("/home/mozhi/Record/video_ts.txt",
                            "/home/mozhi/Record/video.avi");

  double img_time;
  ROSThread thread(imureader, videoreader, cmdreader);
  thread.showVideo = true;
  ros::Rate loop_rate(50);
  ////////////////////////////////
  FindRob find_rob(NULL);
  IplImage *imgsrc;
  CvSize imgSize = {640, 360};
  imgsrc = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
  Mat imgmat;
  
  system("rosservice call /ardrone/setcamchannel 1");
  ///////////////////////// PID control parameters
  double targetx, targety;
  double centerx, centery;
  double errorx, errory, errorturn;
  double lasterrorx, lasterrory, lasterrorturn;
  double targetvx, targetvy, targetv;
  int targeth, targethland = 700;
  double leftr = 0, forwardb = 0, upd = 0, turnleftr = 0;
  double vk = 2.0; // 0.001;
  static double kp = 4.0; // 0.0001;
  static double kd = 150.0;
  static double ki = 0.0;
  double scale = 3;
  static double vkp = 5, vkd = 20, vki = 0;
  ///////////////////////////////////PID调节的初始量///////////////////////////////
  PIDController pidX, pidY, pidZ;
  PIDController pidVX, pidVY, pidVZ, PidW;
  pidX.setParam(kp, ki, kd, 2);
  pidY.setParam(kp, ki, kd, 2);
  pidZ.setParam(kp, ki, kd, 3);
  pidVX.setParam(vkp, vki, vkd, 2);
  pidVY.setParam(vkp, vki, vkd, 2);
  pidVZ.setParam(vkp, vki, vkd, 2);
  PidW.setParam(vkp, 20, vkd, 2);
  targetx = 320, targety = 185;
  //////////////////////////////////////////////////////////
  ofstream log;
  char filename[50];
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.txt", a->tm_mday,
          a->tm_hour, a->tm_min, a->tm_sec);

  log.open(filename);
  if (!log) {
    cout << "cannot open file to log" << endl;
  }
  cout << "Start!" << endl;
  ///////////////////////////////////////////////////////////
  ModeType cur_mode = STOP, next_mode = STOP;
  int frame_count = 0, lostframe = 0;
  ///////////////////////////////////////////////////////////
  double takeoff_time;
  double target_altitude = 2000;
  int pid_stable_count = 0;
  double tf_errorx, tf_errory, tf_errorturn;
  cvNamedWindow("main_window", 1);
  while (ros::ok()) {
    usleep(1000);
    if (videoreader.newframe) {
      cur_mode = cmdreader.GetMode();
      frame_count++;
      lostframe = 0;
      videoreader.newframe = false;
      cout << "Battery:" << thread.navdata.batteryPercent << endl;
      videoreader.getImage(imgmat, img_time);
      *imgsrc = imgmat;
      find_rob.ReInit(imgsrc);
      switch (cur_mode) {
      case START:
        LogCurTime(log);
        log << "START!!! TAKEOFF NOW" << std::endl;
        drone.hover();
        drone.takeOff();
        takeoff_time = (double)ros::Time::now().toSec();
        while ((double)ros::Time::now().toSec() < takeoff_time + 5);
        next_mode = TAKEOFF;
        break;
      case TAKEOFF:
        LogCurTime(log);
        if (thread.navdata.altd < target_altitude) {
          upd = 0.002 * (target_altitude - thread.navdata.altd);
        }
        else if (thread.navdata.altd > target_altitude + 50) {
          upd = 0.002 * (target_altitude + 50 - thread.navdata.altd);
        }
        else {
          upd = 0;
        }
        leftr = 0;
        forwardb = 0;
        CLIP3(-0.2, upd, 0.2);
        turnleftr = 0;

        if (upd == 0) {
          pid_stable_count++;
          if (pid_stable_count >= 4) {
            log << "TakeOff Complete!!! Folowing Robot" << std::endl;
            drone_tf.SetRefPose(0, img_time);
            next_mode = FOLLOWROBOT;
          }
        }
        else {
          pid_stable_count = 0;
          log << "TAKEOFF!! altitude = " << thread.navdataCb.altd << std::endl;
        }
        break;
      case TOCENTER:
        break;
      case TOROBOT:
        break;
      case FOLLOWROBOT:
        LogCurTime(log);
        drone_tf.GetDiff(tf_errorx, tf_errory, tf_errorturn);
        log << "tf_errorx = " << tf_errorx << "tf_errory = " << tf_errory 
            << "tf_errorturn = " << tf_errorgturn << std::endl;

        if (find_rob.doesRobotExist()) {
          centerx = find_rob.getRobCenter.x;
          centery = find_rob.getRobCenter.y;
          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = centerx - targetx;
          errory = centery - targety;
          targetvx = -vk * errory - vk * (errory - lasterrory);
          targetvy = -vk * errorx - vk * (errorx - lasterrorx);
          if (errory > 80 || errory < -80) {
            targetvx += -vk * errory + 80 * vk;
          }
          if (errorx > 80 || errorx < -80) {
            targetvy += -vk * errorx + 80 * vk;
          }
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);
          forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
          leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
          leftr /= 15000;
          forwardb /= 15000;

          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          upd = 0;
          turnleftr = 0;

          if (abs(errorx) < 30 && abs(errory) < 30) {
            errorturn = find_rob.getRobDir();
            turnleftr = errorturn * 10;
            CLIP3(-0.15, turnleftr, 0.15);
            if (abs(errorturn) < 0.1) {
              turnleftr = 0;
              if (pid_stable_count >= 4) {
                log << "Finish Flowing Robot" << std::endl;
              }
            }
            else {
              pid_stable_count = 0;
              log << "TURNING!" << std::endl;
              log << "errorturn = " << errorturn << "turnleftr = " 
                  << turnleftr << std::endl;

            }
          }
          else {
            log << "PID to CENTER" << std::endl;
            log << "errorx = " << errorx << "errory = " << errory << std::endl
                << "forward = " << forwardb << "leftr = " << leftr << std::endl;

          }
        }
        break;
      case SEARCHING:
        break;
      default:
        break;
      }
    }
    lostframe++;
    if (lostframe > 3000) {
      drone.land(); // if the video is not fluent
      continue;
    }
    if (lostframe > 100) {
      cout << "stuck." << endl;
      drone.hover(); // if the video is not fluent
      continue;
    }
    if (cur_mode == cmdreader.GetMode() && cur_mode != MANUL) {
      cmdreader.RunNextMode(next_mode, leftr, forwardb, upd, turnleftr);
    }
    if (cmdreader.GetMode() == MANUL) {
      if (static_cast<double>(clock() - cmdreader.GetManualTime()) /
        CLOCKS_PER_SEC * 1000 >
        1000) {

        drone.hover();
      }
    }
    cvShowImage("a", imgsrc);
    cv::waitKey(1);
  }
  drone.land();
  cvReleaseImage(&imgsrc);
  return 0;
}

void ROSControl_main(int argc, char **argv) {
  // ros::init(argc, argv, "listener");
  pthread_t ROS_thread, control_thread;
  int rc = pthread_create(&control_thread, NULL, Control_loop, 0);
  if (rc) {
    printf("ERROR; return code from pthread_create() is %d\n", rc);
    exit(-1);
  }

  pthread_join(control_thread, NULL);
}

CvANN_MLP bp;
#include "IMUVideoSync.h"
int main(int argc, char **argv) {
#if premode == 1
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel1.xml");
#else
#if premode == 2
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel2.xml");
#else
  bp.load(
    "/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel_op.xml");
#endif
#endif
  ros::init(argc, argv, "ARDrone_test");
  ROSControl_main(argc, argv);
  fout.close();

  return 0;
}