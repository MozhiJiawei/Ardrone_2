
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
#include "PID.h"
#include "ExternalCamera.h"

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

  ExternalCamera ex_cam;
  double img_time;
  ROSThread thread(imureader, videoreader, cmdreader, ex_cam);
  thread.showVideo = true;
  ros::Rate loop_rate(50);
  ////////////////////////////////
  FindRob find_rob(NULL);
  IplImage *imgsrc;
  CvSize imgSize = {640, 360};
  imgsrc = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
  Mat imgmat;
  
  system("rosservice call /ardrone/setcamchannel 1");
  //system("rosservice call /ardrone/flattrim");
  ///////////////////////// PID control parameters
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
  PID pid(thread, find_rob);
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
  double robot_x = 1, robot_y = 0;
  double drone_x, drone_y;
  double targetx = 320, targety = 185;
  double takeoff_altitude = 1200;
  double follow_altitude = 1200;

  double takeoff_time;
  int pid_stable_count = 0;

  double flying_scale = 300;

  double tf_errorx, tf_errory, tf_errorturn;
  cvNamedWindow("a", 1);
  cvMoveWindow("a", 600, 350);
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
      find_rob.doesRobotExist();
      find_rob.isGroundEdge();
      find_rob.doesGroundCenterExist();
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
        if (find_rob.doesGroundCenterExist()) {
          centerx = find_rob.getGroundCenter().x;
          centery = find_rob.getGroundCenter().y;
          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          errory = centery - targety;
          errorx = centerx - targetx;
          forwardb = pid.PIDXY(errory, 1500);
          leftr = pid.PIDXY(errorx, 1500, false);
          upd = pid.PIDZ(takeoff_altitude, 50);
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          turnleftr = 0;

          if (abs(errorx) < 30 && abs(errory) < 30 && upd == 0) {
            pid_stable_count++;
            if (pid_stable_count >= 4) {
              log << "TakeOff Complete!!! Folowing Robot" << std::endl;
              drone_tf.SetRefPose(0, img_time);
              next_mode = TOROBOT;
              pid.PIDReset();
              //next_mode = FOLLOWROBOT;
              //next_mode = SEARCHING;
            }
          }
          else {
            pid_stable_count = 0;
            log << "TAKEOFF!! altitude = " << thread.navdata.altd << std::endl;
            log << "errory = " << errory 
                << "  forwardb = " << forwardb << std::endl;

            log << "errorx = " << errorx << "  leftr = " << leftr << std::endl;
          }
        }
        break;
      case TOCENTER:
        LogCurTime(log);
        drone_tf.GetDiff(drone_x, drone_y, errorturn);
        forwardb = pid.PIDXY(drone_x * flying_scale, 500);
        leftr = pid.PIDXY(drone_y * flying_scale, 500);
        CLIP3(-0.1, leftr, 0.1);
        CLIP3(-0.1, forwardb, 0.1);
        upd = 0;
        turnleftr = 0;
        if (abs(drone_x) < 0.2 && abs(drone_y) < 0.2) {
          if (find_rob.doesGroundCenterExist()) {
            next_mode = TAKEOFF;
            pid.PIDReset();
          }
        }
        break;
      case TOROBOT:
        LogCurTime(log);
        drone_tf.GetDiff(drone_x, drone_y, errorturn);
        errorx = drone_x - robot_x;
        errory = drone_y - robot_y;
        forwardb = pid.PIDXY(errorx * flying_scale, 500);
        leftr = pid.PIDXY(errory * flying_scale, 500, false);
        CLIP3(-0.1, leftr, 0.1);
        CLIP3(-0.1, forwardb, 0.1);
        upd = 0;
        turnleftr = 0;
        if (find_rob.doesRobotExist()) {
          pid_stable_count++;
          if (pid_stable_count >= 3) {
            log << "FIND ROBOT!! Follow it!" << std::endl;
            next_mode = FOLLOWROBOT;
            pid.PIDReset();
          }
        }
        else {
          pid_stable_count = 0;
        }
        log << "errorx = " << errorx << "  forward = " << forwardb << std::endl;
        log << "errory = " << errory << "  leftr = " << leftr << std::endl;
        break;
      case FOLLOWROBOT:
        LogCurTime(log);
        drone_tf.GetDiff(tf_errorx, tf_errory, tf_errorturn);
        log << "tf_errorx = " << tf_errorx << "tf_errory = " << tf_errory
          << "tf_errorturn = " << tf_errorturn << std::endl;

        if (find_rob.doesRobotExist()) {
          centerx = find_rob.getRobCenter().x;
          centery = find_rob.getRobCenter().y;
          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          errory = centery - targety;
          errorx = centerx - targetx;
          forwardb = pid.PIDXY(errory, 1000);
          leftr = pid.PIDXY(errorx, 1000, false);
          upd = pid.PIDZ(90, 10, false);
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          turnleftr = 0;

          if (abs(errorx) < 50 && abs(errory) < 50) {
            //errorturn = find_rob.getRobDir();
            //turnleftr = errorturn * 10;
            //CLIP3(-0.15, turnleftr, 0.15);
            forwardb = 0;
            leftr = 0;
            if (abs(errorturn) < 0.1 && upd == 0) {
              turnleftr = 0;
              if (pid_stable_count >= 4) {
                log << "Finish Flowing Robot" << std::endl;
                next_mode = TOCENTER;
                pid.PIDReset();
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
            log << "errorx = " << errorx << " errory = " << errory << std::endl
                << "forward = " << forwardb << " leftr = " << leftr << std::endl;

            log << "altd = " << thread.navdata.altd << std::endl;
            log << "RobRadius = " << find_rob.getRobRadius() << std::endl;
            log << "upd = " << upd << std::endl;
          }
        } else {
          upd = 0;
          // TOROBOT.
        }
        break;
      case SEARCHING:
        LogCurTime(log);
        drone_tf.GetDiff(tf_errorx, tf_errory, tf_errorturn);
        log << "vx = " << thread.navdata.vx << "vy = " << thread.navdata.vy 
            << std::endl;

        log << "tf_errorx = " << tf_errorx << "  tf_errory = " << tf_errory 
            << "  tf_errorturn = " << tf_errorturn << std::endl;

        forwardb = 0.05;
        leftr = 0;
        upd = 0;
        turnleftr = 0;
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
