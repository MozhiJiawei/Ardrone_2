/*
 * main.cpp
 *
 *  Created on: May 14, 2015
 *      Author: mrh
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

#include "ROSThread.h"
#include "VideoRecorder.h"
#include "IMURecorder.h"
#include "ImgRecon.h"
#include "GridDetector.h"
#include "PIDController.h"
#include "AffineTransform.h"
#include "PredictNumber.h"
#include "SearchNumber.h"
#include "CMDReciever.h"
#include "time.h"

#include <opencv2/opencv.hpp>
#include <vector>
 ///////////////////////////////////
 //#include <ardrone_autonomy/Navdata.h>
 //#include <std_msgs/Empty.h>
 //#include <geometry_msgs/Twist.h>
 //#include "precomp.hpp"
#include "opencv2/legacy/blobtrack.hpp"
#include "math.h"
#define PI 3.14159265

#include <ros/ros.h>
#include <keyboard/Key.h>
using namespace std;

#define Test 0
#define MyCode 1
#define MyPIDMode 0

static int mGrids = 5;
static int nGrids = 6;
static int setcamera = 0;
//////////////////////////////////
/*geometry_msgs::Twist twist_msg;
  geometry_msgs::Twist twist_msg_neg;
  geometry_msgs::Twist twist_msg_hover;
  geometry_msgs::Twist twist_msg_up;
  std_msgs::Empty emp_msg;*/
  /////////////////////////////////




void writeFeatPts(const vector<cv::Point2f>& featPts, const char* filePath) {
  ofstream file(filePath);
  if (!file)
    repErr("cannot open '%s' to write!", filePath);

  for (size_t i = 0; i < featPts.size(); i++) {
    file << featPts[i].x << " " << featPts[i].y << endl;
  }
}

void writeIMUMeas(const vector<IMUData>& imumeas, const char* filePath) {
  std::ofstream file(filePath);
  if (!file)
    repErr("cannot open '%' to write!", filePath);

  for (size_t i = 0; i < imumeas.size(); i++) {
    imumeas[i].write(file);
  }
  file.close();
}

#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
ofstream fout("/home/mozhi/Record/test.txt", ios::app);
#include "ARDrone.h"

void* Control_loop(void* param) {
  ARDrone drone;
  drone.setup();
  CMDReciever cmdreader("/home/mozhi/Logs/cmd.txt", drone);
  IMURecorder imureader("/home/mozhi/Record/imu.txt");
  VideoRecorder videoreader("/home/mozhi/Record/video_ts.txt", "/home/mozhi/Record/video.avi");
  ROSThread thread(imureader, videoreader, cmdreader);
  thread.showVideo = true;
  ros::Rate loop_rate(50);
  ////////////////////////////////
  ImgRecon img_recon(NULL);
  ImgRGB img(640, 360);
  IplImage *imgsrc, *imgr, *imgg, *imgb, *squaretmp, *squaretmp1, *imgyellow, *imgnumber, *imgnumberwarp;
  CvSize imgSize = { 640,360 }, imgnumbersize = { 128,128 };
  imgr = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
  imgyellow = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
  imgsrc = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
  imgnumberwarp = cvCreateImage(imgnumbersize, IPL_DEPTH_8U, 3);
  imgg = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
  imgb = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
  squaretmp = cvLoadImage("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bz1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
  squaretmp1 = cvLoadImage("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/7.jpg", 1);

  system("rosservice call /ardrone/setcamchannel 1");
  ////////////////////////////////
  int i, j;
  uchar * data, *datar, *datag, *datab;
  int pathJustLost = 0, pathLostCount = 0;
  double lasttargetx = 300, lasttargety = 180;
  int blobSize = 0, targetBlobSize = 2000;
  int controlMode = 0, controlflag = 0; //0: Manual; 1: Auto // controlMode作为对不同飞行状态的选择
  PIDController pidX, pidY, pidZ;
  PIDController pidVX, pidVY, pidVZ, PidW;
  int frame_count = 0, lostframe = 0;
  double lastheadx = 600, lastheady = 180;
  double rightheadx = 600, rightheady = 180;
  double redAveY = 0;
  //CvBlobSeq* pOldBlobList=NULL;
  char c;
  ///////////////////////// PID control parameters
  double targetx, targety;
  double centerx, centery, centerxavr, centeryavr, centerxblue, centeryblue;
  double errorx, errory, errorturn;
  double lasterrorx, lasterrory, lasterrorturn;
  double targetvx, targetvy, targetv;
  int targeth, targethland = 700;
  double leftr = 0, forwardb = 0, upd = 0, leftrmode4 = -0.1, forwardbmode4 = 0, updmode4 = 0, turnleftr = 0, lastturnleftr = 0, turnaa = 0;
  double vk = 2.0;//0.001;
  static double kp = 4.0;//0.0001;
  static double kd = 150.0;
  static double ki = 0.0;
  double scale = 3;
  static double vkp = 5, vkd = 20, vki = 0;
  ///////////////////////////find number//////////////project 2///////////
  float yellowpercent;
  int pre, ready = 0;
  CvRect omegasquare;
  int leftside = 1000, rightside = 0, upside = 1000, downside = 0;
  vector<vector<cv::Point> > squares;
  Mat imgmat, imgmatyellow;
  number mynumner;
  int currentnumber = 1, countsum = 0, numberpositiony = 0, numberpositionx = 0;
  int countnumber[10];
  CvMat* warp_mat = cvCreateMat(3, 3, CV_32FC1);
  clock_t land_takeoff_time = clock();
  clock_t forward_time = clock();
  clock_t up_time = clock();
  clock_t find_time = clock();
  clock_t test_time = clock();
  double velNum[10][2] = { 0 };//v[0]->水平,v[1]->垂直 分别代表像9个停机坪飞行的X，Y方向上速度，默认值
  velNum[0][0] = 0, velNum[0][1] = 0;    //0
  velNum[1][0] = -0.1, velNum[1][1] = 0.05;   //1
  velNum[2][0] = -0.12, velNum[2][1] = 0.02;     //2
  velNum[3][0] = 0, velNum[3][1] = 0.2;     //3
  velNum[4][0] = 0.08, velNum[4][1] = 0.1;   //4
  velNum[5][0] = 0.1, velNum[5][1] = -0.08;     //5
  velNum[6][0] = -0.05, velNum[6][1] = 0.1;    //6
  velNum[7][0] = 0.1, velNum[7][1] = 0.02;     //7
  velNum[8][0] = -0.1, velNum[8][1] = 0;     //8
  velNum[9][0] = 0, velNum[9][1] = 0.1;     //9
  int testnumbernow = 1;
  int mode5start = 0;
  int mode5clock = 0;
  double time_fly[10] = { 0,6000.91,6000.59, 3500.75, 5500.7,9000.87,10500.9,9372.34, 4955.11,7466.54 }; //有一个停机坪到下一个停机坪的飞行时间，默认值
  int mode3findflag = 1;
  int pre_count = 0;
  ///////////////////////////////////PID调节的初始量///////////////////////////////
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
  sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.txt",
    a->tm_mday, a->tm_hour, a->tm_min, a->tm_sec);

  log.open(filename);
  if (!log) {
    cout << "cannot open file to log" << endl;
  }
  cout << "Start!" << endl;

  ///////////////////////////////////////////////////////////
  ModeType cur_mode = STOP, next_mode = STOP;
  ///////////////////////////////////////////////////////////
  clock_t pid_stable_time;
  clock_t landing_time;
#if Test
  drone.takeOff();
  ros::spin();
#endif
  cvNamedWindow("a", 1);
  while (ros::ok()) {
    usleep(1000);
    lostframe++;
    if (c == 'x')  drone.land();
    if (lostframe > 100) drone.hover(); // if the video is not fluent
    if (lostframe > 3000) drone.land(); // if the video is not fluent
    //////////////////////////test/////////////////////////////////////////////////
#if MyCode 
    if (videoreader.newframe) {
      cur_mode = cmdreader.GetMode();
      if (cur_mode == START) {
        cout << "START";
      }
      frame_count++;
      lostframe = 0;
      videoreader.newframe = false;
      cout << "Battery:" << thread.navdata.batteryPercent << endl;

      videoreader.getImage(imgmat);
      *imgsrc = imgmat;
          img_recon.ReInit(imgsrc);
      switch (cur_mode) {
      case START:
        drone.takeOff();
        leftr = 0;
        forwardb = 0;
        upd = 0;
        turnleftr = 0;
        next_mode = TAKEOFF;
        break;
      case TAKEOFF:
        if (thread.navdata.altd > 400)
        {
          time(&timep);
          a = localtime(&timep);
          log << endl << a->tm_mday << " " << a->tm_hour << ":"
            << a->tm_min << ":" << a->tm_sec << endl;

          img_recon.ReInit(imgsrc);
          if (img_recon.ContourExist()) {
            cout << "conter found" << endl;
            centerx = img_recon.GetCenterPoint().x;
            centery = img_recon.GetCenterPoint().y;
            log << "center founded: x = " << centerx
              << "  y = " << centery << endl;

#if MyPIDMode
            CLIP3(10.0, centerx, 590.0);
            CLIP3(10.0, centery, 350.0);
            lasterrorx = errorx;
            lasterrory = errory;
            errorx = centerx - targetx;
            errory = centery - targety;
            // control vx
            if (abs(errorx) < 5) {
              leftr = 0;
            }
            else {
              leftr = -0.0025 * errorx - 0.0025 *(errorx - lasterrorx);
            }
            // control vy
            if (abs(errory) < 5) {
              forwardb = 0;
            }
            else {
              forwardb = -0.0025 * errory - 0.0025*(errory - lasterrory);
            }
            // control vz
            if (thread.navdata.altd < 1400) {
              upd = 0.002 * (1400 - thread.navdata.altd);
            }
            else if (thread.navdata.altd > 1450) {
              upd = 0.002 * (1450 - thread.navdata.altd);
            }
            else {
              upd = 0;
            }

            if (upd == 0 && leftr == 0 && forwardb == 0) {
              if ((clock() - pid_stable_time) / CLOCKS_PER_SEC * 1000 
                  > 1000) {

                log << "TAKEOFF Complete! Ready to land!" << endl;
                next_mode = LAND;
              }
              log << "PID Complete" << endl;
            }
            else {
              CLIP3(-0.1, leftr, 0.1);
              CLIP3(-0.1, forwardb, 0.1);
              CLIP3(-0.2, upd, 0.2);
              turnleftr = 0;
              pid_stable_time = clock();
              log << "e_x = " << errorx << "  e_y = " << errory << endl;
              log << " Height:" << setw(8) << thread.navdata.altd << "  v: "
                << setw(8) << thread.navdata.vy << " " << setw(8)
                << thread.navdata.vx << endl;//<<" "<< thread.navdata.vz;
              log << "y_left = " << leftr << "  x_forward = " << forwardb
                << "  z_up = " << upd << endl;
            }
#else
            CLIP3(10.0, centerx, 590.0);
            CLIP3(10.0, centery, 350.0);
            lasterrorx = errorx;
            lasterrory = errory;
            errorx = centerx - targetx;
            errory = centery - targety;
            log << "e_x = " << errorx << "  e_y = " << errory << endl;
            targetvx = -vk * errory - vk*(errory - lasterrory);
            targetvy = -vk * errorx - vk*(errorx - lasterrorx);
            if (errory > 80 || errory < -80) {
              targetvx += -vk * errory + 80 * vk;
            }
            if (errorx > 80 || errorx < -80) {
              targetvy += -vk * errorx + 80 * vk;
            }
            CLIP3(-1500.0, targetvx, 1500.0);
            CLIP3(-1500.0, targetvy, 1500.0);
            log << "t_vx = " << targetvx << "  t_vy = " << targetvy << endl;
            log << " Height:" << setw(8) << thread.navdata.altd << "  v: "
              << setw(8) << thread.navdata.vx << " " << setw(8)
              << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;

            forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
            leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
            leftr /= 15000;        forwardb /= 15000;
            if (thread.navdata.altd < 1400) {
              upd = 0.002 * (1400 - thread.navdata.altd);
            }
            else if (thread.navdata.altd > 1450) {
              upd = 0.002 * (1450 - thread.navdata.altd);
            }
            else {
              upd = 0;
            }
            if (abs(errorx) < 15 && abs(errory) < 15) {
              leftr = 0;
              forwardb = 0;
              if (upd == 0) {

                if ((clock() - pid_stable_time) / CLOCKS_PER_SEC * 1000
                      > 1000) {

                  log << "TAKEOFF Complete!";
                  next_mode = LAND;
                }
                log << "PID Complete";
              }
            }
            else {
              CLIP3(-0.1, leftr, 0.1);
              CLIP3(-0.1, forwardb, 0.1);
              CLIP3(-0.2, upd, 0.2);
              turnleftr = 0;
              pid_stable_time = clock();
              log << "y_left = " << leftr << "  x_forward = " << forwardb
                << "  z_up = " << upd << endl;
            }
#endif
          }
        }
        break;
      case LAND: 
        time(&timep);
        a = localtime(&timep);
        log << endl << a->tm_mday << " " << a->tm_hour << ":"
          << a->tm_min << ":" << a->tm_sec << endl;

        img_recon.ReInit(imgsrc);
        if (img_recon.ContourExist()) {
          cout << "conter found" << endl;
          centerx = img_recon.GetCenterPoint().x;
          centery = img_recon.GetCenterPoint().y;
          log << "center founded: x = " << centerx
            << "  y = " << centery << endl;

#if MyPIDMode
          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = targetx - centerx;
          errory = targety - centery;
          // control vx
          if (abs(errorx) < 5) {
            leftr = 0;
          }
          else {
            leftr = -0.0025 * errory - 0.0025 *(errory - lasterrory);
          }
          // control vy
          if (abs(errory) < 5) {
            forwardb = -0.0025 * errorx - 0.0025*(errorx - lasterrorx);
          }
          else {
            forwardb = 0;
          }
          // control vz
          if (thread.navdata.altd < 350) {
            upd = 0.002 * (350 - thread.navdata.altd);
          }
          else if (thread.navdata.altd > 400) {
            upd = 0.002 * (400 - thread.navdata.altd);
          }
          else {
            upd = 0;
          }

          if (upd == 0 && leftr == 0 && forwardb == 0) {
            if ((clock() - pid_stable_time) / CLOCKS_PER_SEC * 1000
                > 1000) {

              log << "Ready to land!" << endl;
              drone.land();
              landing_time = clock();
              while (static_cast<double>(clock() - landing_time)
                  / CLOCKS_PER_SEC * 1000 < 5000);

              next_mode = START;
            }
            log << "PID Complete" << endl;
          }
          else {
            CLIP3(-0.1, leftr, 0.1);
            CLIP3(-0.1, forwardb, 0.1);
            CLIP3(-0.2, upd, 0.2);
            turnleftr = 0;
            pid_stable_time = clock();
            log << "e_x = " << errorx << "  e_y = " << errory << endl;
            log << " Height:" << setw(8) << thread.navdata.altd << "  v: "
              << setw(8) << thread.navdata.vy << " " << setw(8)
              << thread.navdata.vx << endl;//<<" "<< thread.navdata.vz;
            log << "y_left = " << leftr << "  x_forward = " << forwardb
              << "  z_up = " << upd << endl;
          }

#endif
        }
        break;
      default:
        break;
      }
      c = (char)cv::waitKey(1);
      // ESC key pressed
      if (c == 27 || c == 'q') { drone.land(); break; }
      if (c > -1) cout << " key press: " << (int)c << endl;
    }
#else
    if (videoreader.newframe) {// new frame?
      frame_count++;
      cout << lostframe << "ms ";
      lostframe = 0;
      videoreader.newframe = false;

      cout << "Battery:" << thread.navdata.batteryPercent << "% Rotate: "
        << setw(8) << thread.navdata.rotX << " " << setw(8)
        << thread.navdata.rotY << " ";//<<setw(12) << thread.navdata.rotZ;

      cout << " Height:" << setw(8) << thread.navdata.altd << "  v: "
        << setw(8) << thread.navdata.vx << " " << setw(8)
        << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;

      if (!videoreader.curImg.empty()) {
        cloneImg(videoreader.curImg, img); //克隆当前相机照片到处理照片
        bgr2rgb(img);
        if (videoreader._recording) {
          drawPoint(img, 15, 15, 6, 255, 0, 0, 2); //统计照片像素点
        }
      }
      videoreader.getImage(imgmat);
      videoreader.getImage(imgmatyellow);
      *imgsrc = imgmat;
      *imgyellow = imgmatyellow;
      if (controlMode != 3) {
        EqualizeHistColorImage(imgyellow);
      }
      yellowpercent = Color_Detection_Pro(imgyellow, centerxavr,
        centeryavr, centerxblue, centeryblue, controlMode); // where is tis?

      findSquares(imgmatyellow, squares, controlMode);
      drawSquares(imgmat, squares);
      cvShowImage("imgyellow", imgyellow);
      if (controlMode == 4) //对不同停机坪数字的处理，水平飞行处理
      {
        //if(mynumner.isNumFind(currentnumber))controlMode=403;
        if (setcamera == 1) {
          system("rosservice call /ardrone/setcamchannel 1");
          setcamera = 0;
        }
        if (currentnumber == 6 || currentnumber == 4 || currentnumber == 2)
        {
          if (static_cast<double>(clock() - find_time) / CLOCKS_PER_SEC * 1000 > 3 * time_fly[currentnumber] / 4)
          {
            if (yellowpercent > 0.1)
            {
              controlMode = 402;
            }
          }
          else if (static_cast<double>(clock() - find_time) / CLOCKS_PER_SEC * 1000 > time_fly[currentnumber])
          {
            controlMode = 402;
          }
        }
        else
        {
          if (static_cast<double>(clock() - find_time) / CLOCKS_PER_SEC * 1000 > 2 * time_fly[currentnumber] / 3)
          {
            if (yellowpercent > 0.1)
            {
              controlMode = 402;
            }
          }
          else if (static_cast<double>(clock() - find_time) / CLOCKS_PER_SEC * 1000 > time_fly[currentnumber])
          {
            controlMode = 402;
          }
        }
        leftr = velNum[currentnumber][0];
        forwardb = velNum[currentnumber][1];
        upd = 0;

      }
      else if (controlMode == 400) //对准目标停机坪中心，利用PID调节
      {
        //cvErode(imgyellow,imgyellow,0,1);//腐蚀
        //cvDilate(imgyellow,imgyellow,0,1);//膨胀
        if (squares.size() > 0)
        {
          centerxavr = 0;
          centeryavr = 0;
        }
        for (i = 0; i < squares.size(); i++)
        {
          centerx = 0;
          centery = 0;
          for (j = 0; j < 4; j++)
          {
            if (squares[i][j].x < leftside)leftside = squares[i][j].x;

            if (squares[i][j].y < upside)upside = squares[i][j].y;
            if (squares[i][j].x > rightside)rightside = squares[i][j].x;
            if (squares[i][j].y > downside)downside = squares[i][j].y;
            centerx += squares[i][j].x;
            centery += squares[i][j].y;
          }
          centerx /= 4;
          centery /= 4;
          centerxavr += centerx;
          centeryavr += centery;
        }
        if (squares.size() > 0)
        {
          centerxavr /= squares.size();
          centeryavr /= squares.size();
          CLIP3(10.0, centerxavr, 590.0);
          CLIP3(10.0, centeryavr, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = centerxavr - targetx;
          errory = centeryavr - targety;
          targetvx = -vk * errory - vk*(errory - lasterrory);
          targetvy = -vk * errorx - vk*(errorx - lasterrorx);
          if (errory > 80 || errory < -80) targetvx += -vk * errory + 80 * vk;
          if (errorx > 80 || errorx < -80) targetvy += -vk * errorx + 80 * vk;
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);
          forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
          leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
          leftr /= 15000;        forwardb /= 15000;
          if (yellowpercent > 0.28) upd = 10 * (yellowpercent - 0.28);
          if (yellowpercent < 0.26) upd = -10 * (0.26 - yellowpercent);
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          omegasquare.x = leftside;
          omegasquare.y = upside;
          omegasquare.width = rightside - leftside;
          omegasquare.height = downside - upside;
          leftside = 1000, rightside = 0, upside = 1000, downside = 0;
          if (abs(errorx) < 15 && abs(errory) < 15)
          {
            cout << "test_targeth" << targeth << endl;
            targeth = thread.navdata.altd;
            warpPerspective(imgyellow, imgnumberwarp, squares[0]);
            cvShowImage("testprenum4", imgnumberwarp);
            pre = predictNumber(imgnumberwarp, 8);
            cout << "predictnum:" << setw(8) << pre << endl;
            if (pre > -1)++countnumber[pre];
            ++countsum;
            if (countsum > 2)
            {
              //controlMode=405;
              //controlflag=0;
              if (1.0*countnumber[currentnumber] / countsum > 0.5)
              {
                controlMode = 405;
                controlflag = 0;
                mynumner.add(currentnumber, numberpositionx, numberpositiony);
                if (leftrmode4 > 0) numberpositionx++;
                else numberpositionx--;
                for (int l = 0; l < 10; l++)
                {
                  countnumber[l] = 0;
                }
              }
              else
              {
                for (int i = 0; i < 10; i++)
                {
                  if (countnumber[i] / countsum > 0.8)
                  {
                    mynumner.add(i, numberpositionx, numberpositiony);
                    if (leftrmode4 > 0) numberpositionx++;
                    else numberpositionx--;
                    controlMode = 4;
                    controlflag = 0;
                    for (int l = 0; l < 10; l++)
                    {
                      countnumber[l] = 0;
                    }
                  }
                }
              }
            }

          }
        }
        else
        {
          CLIP3(10.0, centerxavr, 590.0);
          CLIP3(10.0, centeryavr, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = centerxavr - targetx;
          errory = centeryavr - targety;
          targetvx = -vk * errory - vk*(errory - lasterrory);
          targetvy = -vk * errorx - vk*(errorx - lasterrorx);
          if (errory > 80 || errory < -80) targetvx += -vk * errory + 80 * vk;
          if (errorx > 80 || errorx < -80) targetvy += -vk * errorx + 80 * vk;
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);
          forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, scale);
          leftr = pidVY.getOutput(targetvy - thread.navdata.vy, scale);
          leftr /= 15000;        forwardb /= 15000;
          if (yellowpercent > 0.28) upd = 10 * (yellowpercent - 0.28);
          if (yellowpercent < 0.26) upd = -10 * (0.26 - yellowpercent);
          if (yellowpercent<0.08)upd = 0.1;
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          if (thread.navdata.altd - targeth>-20)upd = 0.2;
        }
        //imshow("testprenum",imgmatyellow);
      }
      else if (controlMode == 401)
      {
        if (yellowpercent < 0.1)controlMode == 402;
        if (ready == 0)
        {
          ready = 1;
          /*else
            {
            upd=0.2;
            leftr=0;
            forwardb=0;
            drone->move(0, 0, 0.2, 0);
            cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<" "<<thread.navdata.altd<<" "<<targeth<<endl;
            }*/
        }
        if (ready == 1)
        {
          if (squares.size()>0)
          {
            centerxavr = 0;
            centeryavr = 0;
          }
          for (i = 0; i < squares.size(); i++)
          {
            centerx = 0;
            centery = 0;
            for (j = 0; j < 4; j++)
            {
              if (squares[i][j].x < leftside)leftside = squares[i][j].x;
              if (squares[i][j].y < upside)upside = squares[i][j].y;
              if (squares[i][j].x > rightside)rightside = squares[i][j].x;
              if (squares[i][j].y > downside)downside = squares[i][j].y;
              centerx += squares[i][j].x;
              centery += squares[i][j].y;
            }
            centerx /= 4;
            centery /= 4;
            centerxavr += centerx;
            centeryavr += centery;
          }
          if (squares.size() > 0)
          {
            centerxavr /= squares.size();
            centeryavr /= squares.size();
            CLIP3(10.0, centerxavr, 590.0);
            CLIP3(10.0, centeryavr, 350.0);
            lasterrorx = errorx;
            lasterrory = errory;
            errorx = centerxavr - targetx;
            errory = centeryavr - targety;
            targetvx = -vk * errory - vk*(errory - lasterrory);
            targetvy = -vk * errorx - vk*(errorx - lasterrorx);
            if (errory > 80 || errory < -80) targetvx += -vk * errory + 80 * vk;
            if (errorx > 80 || errorx < -80) targetvy += -vk * errorx + 80 * vk;
            CLIP3(-1500.0, targetvx, 1500.0);
            CLIP3(-1500.0, targetvy, 1500.0);
            forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
            leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
            leftr /= 15000;        forwardb /= 15000;
            if (yellowpercent > 0.3) upd = 10 * (yellowpercent - 0.3);
            if (yellowpercent < 0.28) upd = -10 * (0.28 - yellowpercent);
            CLIP3(-0.1, leftr, 0.1);
            CLIP3(-0.1, forwardb, 0.1);
            CLIP3(-0.2, upd, 0.2);
            omegasquare.x = leftside;
            omegasquare.y = upside;
            omegasquare.width = rightside - leftside;
            omegasquare.height = downside - upside;
            leftside = 1000, rightside = 0, upside = 1000, downside = 0;
            if (abs(errorx) < 20 && abs(errory) < 20)
            {
              pre_count++;
              if (pre_count > 1)
              {
                controlMode = 4;
                find_time = clock();
                pre_count = 0;
              }
            }
          }
          else
          {
            CLIP3(10.0, centerxavr, 590.0);
            CLIP3(10.0, centeryavr, 350.0);
            lasterrorx = errorx;
            if (!video)
              lasterrory = errory;
            errorx = centerxavr - targetx;
            errory = centeryavr - targety;
            targetvx = -vk * errory - vk*(errory - lasterrory);
            targetvy = -vk * errorx - vk*(errorx - lasterrorx);
            if (errory > 80 || errory < -80) targetvx += -vk * errory + 80 * vk;
            if (errorx > 80 || errorx < -80) targetvy += -vk * errorx + 80 * vk;
            CLIP3(-1500.0, targetvx, 1500.0);
            CLIP3(-1500.0, targetvy, 1500.0);
            forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
            leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
            leftr /= 15000;        forwardb /= 15000;
            if (yellowpercent > 0.3) upd = 10 * (yellowpercent - 0.3);
            // if (yellowpercent < 0.28) upd = - 10*(0.28-yellowpercent);
            CLIP3(-0.1, leftr, 0.1);
            CLIP3(-0.1, forwardb, 0.1);
            CLIP3(-0.2, upd, 0.2);
            if (thread.navdata.altd - targeth > -20)upd = 0.2;


          }
        }

      }
      else if (controlMode == 402)
      {

        if (yellowpercent > 0.15)
        {
          for (int i = 0; i < 10; i++)countnumber[i] = 0;
          countsum = 0;
          controlMode = 400;
          controlflag = 0;
          forwardbmode4 = 0;
          leftr = 0;
          forwardb = 0;
        }
        else
        {
          if (thread.navdata.altd < 1600)upd = 0.2;
          else upd = 0;
        }
      }
      else if (controlMode == 403)
      {
        //到已知数字的位置
        controlMode = 400;
      }
      else if (controlMode == 404)//未知场地使用，走偏处理
      {
        //前进一格,飞行0.5秒
        forwardb = forwardbmode4;
        clock_t clock_tmp = clock();
        if (static_cast<double>(clock_tmp - forward_time) / CLOCKS_PER_SEC * 1000>500)
        {
          controlMode = 4;
          forwardbmode4 = 0;
        }
        else
        {
          clock_tmp = clock();
        }
      }
      else if (controlMode == 405)
      {
        //锁定数字降落
        drone.land();
        land_takeoff_time = clock();
        while (thread.navdata.altd != 0);
        land_takeoff_time = clock();
        drone.takeOff();
        while (static_cast<double>(clock() - land_takeoff_time) / CLOCKS_PER_SEC * 1000 < 6000);
        land_takeoff_time = clock();
        while (static_cast<double>(clock() - land_takeoff_time) / CLOCKS_PER_SEC * 1000 < 1000);
        {
          if (thread.navdata.altd - targeth > -20)
          {
            drone.moveUp((float) 0.2);
          }
        }
        if (!(drone.isFlying))
        {
          drone.takeOff();
          land_takeoff_time = clock();
          while (static_cast<double>(clock() - land_takeoff_time) / CLOCKS_PER_SEC * 1000 < 4000);
        }
        ++currentnumber;
        controlMode = 401;
        ready = 0;
        up_time = clock();
        upd = 0.1;
      }
      else if (controlMode == 3)
      {
        if (setcamera == 0) {
          system("rosservice call /ardrone/setcamchannel 2");
          setcamera = 1;
        }
        if (squares.size() > 0)
        {
          centerxavr = 0;
          centeryavr = 0;
        }
        for (i = 0; i < squares.size(); i++)
        {
          centerx = 0;
          centery = 0;
          for (j = 0; j < 4; j++)
          {
            if (squares[i][j].x < leftside)leftside = squares[i][j].x;
            if (squares[i][j].y < upside)upside = squares[i][j].y;
            if (squares[i][j].x > rightside)rightside = squares[i][j].x;
            if (squares[i][j].y > downside)downside = squares[i][j].y;
            centerx += squares[i][j].x;
            centery += squares[i][j].y;
          }
          centerx /= 4;
          centery /= 4;
          centerxavr += centerx;
          centeryavr += centery;
        }
        if (squares.size() > 0)
        {
          omegasquare.x = leftside;
          omegasquare.y = upside;
          omegasquare.width = rightside - leftside;
          omegasquare.height = downside - upside;
          //findones returns the ones of omegasquare in imageyellow
          yellowpercent = 1.0*findones(imgyellow, omegasquare) / (imgSize.height*imgSize.width);
          cout << "yellowpercent_findones" << yellowpercent << endl;
          warp_mat = warpPerspective(imgyellow, imgnumberwarp, squares[0]);
          cvShowImage("testprenum4", imgnumberwarp);
          centerxavr /= squares.size();
          centeryavr /= squares.size();
          CLIP3(10.0, centerxavr, 590.0);
          CLIP3(10.0, centeryavr, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          lasterrorturn = errorturn;
          errorturn = targetx - centerxavr;
          //errorturn=0;
          cout << "errorturn" << errorturn << endl;
          errory = (yellowpercent - 0.2) * 100;
          if (mode3findflag != 1)
          {
            if (errory > 0)errory = 0;
            errorx = 0;
          }
          else
          {
            errorx = centerxavr - targetx;
          }
          cout << "test_yellowpercent:" << yellowpercent << endl;
          // errorturn = - vk * errorturn- vk*(errorturn-lasterrorturn);
          targetvx = -vk * errory - vk*(errory - lasterrory);
          targetvy = -vk * errorx - vk*(errorx - lasterrorx);
          if (errory > 80 || errory < -80) targetvx += -vk * errory + 80 * vk;
          if (errorx > 80 || errorx < -80) targetvy += -vk * errorx + 80 * vk;
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);
          cout << "test_targetvy:" << targetvy << endl;
          forwardb = 2.5*targetvx;
          if (mode3findflag == 1)
          {
            leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.8);
            turnleftr = 0;
          }
          else
          {
            if (fabs(errorturn) < 5)errorturn = 0;
            if (errorturn != 0)
            {
              turnleftr = errorturn / 100;
            }
            else
            {
              turnleftr = 0;
            }
          }

          leftr /= 15000;        forwardb /= 800;
          upd = 0;
          CLIP3(-0.2, leftr, 0.2);
          CLIP3(-0.2, forwardb, 0.2);
          CLIP3(-0.2, upd, 0.2);
          CLIP3(-0.4, turnleftr, 0.4);
          cout << endl;
          if (mode3findflag == 1) cout << "test_forwardb_turnleftr:" << forwardb << " " << leftr << endl;
          else  cout << "test_forwardb_leftr:" << forwardb << " " << turnleftr << endl;
          leftside = 1000, rightside = 0, upside = 1000, downside = 0;
        }
        else
        {
          cout << "失去目标！！！！！！" << endl;
          leftr = 0; forwardb = 0; upd = 0; turnleftr = 0;

        }

      }
      else if (controlMode == 5)
      {
        if (mode5start == 1)

        {
          leftr = velNum[testnumbernow][0];
          forwardb = velNum[testnumbernow][1];
          upd = 0;
          cout << "testnumbernow" << " " << testnumbernow << endl;
        }
        else
        {
          leftr = 0; forwardb = 0; upd = 0;
        }
      }
      else

      {
        leftr = 0; forwardb = 0; upd = 0;
      }
      data = img.data;
      datar = (uchar*)imgr->imageData;
      datag = (uchar*)imgg->imageData;
      datab = (uchar*)imgb->imageData;
      ///////////////////////////////////////////////
      // c=-1;
      c = (char)cv::waitKey(1);
      // ESC key pressed
      if (c == 27 || c == 'q') { drone.land(); break; }
      if (c > -1) cout << " key press: " << (int)c << endl;
    }
#endif
#if MyCode
    if (cur_mode == cmdreader.GetMode() && cur_mode != MANUL) {
      cmdreader.RunNextMode(next_mode, leftr, forwardb, upd,
        turnleftr);

    }
    if (cmdreader.GetMode() == MANUL) {
      if (static_cast<double>(clock() - cmdreader.GetManualTime()) 
          / CLOCKS_PER_SEC * 1000 > 1000) {

        drone.hover();
      }
    }
    cvShowImage("a", imgsrc);
    
#endif
    switch (c) {
    case 'z':
      drone.takeOff();
      break;

    case 'x':
      drone.land();
      next_mode = STOP;
      cmdreader.SetMode(next_mode);
      break;

    case 'c':
      drone.resumeNormal();
      break;

    case 'v':
      drone.emergency();
      break;

    case 'h':
      drone.hover();
      break;

    case 'i':
      drone.moveUp((float) 0.2);
      break;

    case 'k':
      drone.moveDown((float) 0.2);
      break;

    case 'j':
      drone.turnLeft((float) 0.4);
      break;

    case 'l':
      drone.turnRight((float) 0.4);
      break;

    case 'a':
      drone.moveLeft((float) 0.2);
      break;

    case 'd':
      drone.moveRight((float) 0.2);
      break;

    case 'w':
      drone.moveForward((float) 0.2);
      break;

    case 's':

      drone.moveBackward((float) 0.2);
      break;
    case 't':
      if (controlMode == 3)
      {
        mode3findflag = 1 - mode3findflag;
      }
      if (controlMode == 5)
      {
        if (mode5start == 0)
        {
          mode5start = 1;
          test_time = clock();
        }
      }
      break;
    case 'y':
      if (controlMode == 5)
      {
        if (mode5start == 1) {
          mode5start = 0;
          fout << testnumbernow << " " << static_cast<double>(clock() - test_time) / CLOCKS_PER_SEC * 1000 << endl;
        }
      }
      break;
    case 'p':
      if (controlMode == 5)
      {
        // c=-1;
        char o = -1;
        while (o == -1)o = (char)cv::waitKey(33);
        testnumbernow = o - '0';
        cout << "testnumbernow:" << " " << testnumbernow << endl;
      }
      break;
    case '0':
      controlMode = 0;
      break;
    case '1':
      controlMode = 1;
      pidX.reset();
      pidY.reset();
      pidZ.reset();
      break;
    case '2':
      controlMode = 2;
      break;
    case '3':
      controlMode = 3;
      lastheadx = 300;
      lastheady = 5;
      break;
    case '4':
      controlMode = 4;
      find_time = clock();
      break;
    case '5':
      controlMode = 5;
      break;
    case '7':
      controlMode = 7;
      // targetBlobSize= blobSize;
      break;
    case '8':
      controlMode = 8;
      break;

    default:
#if !MyCode
      switch (controlMode) {
      case 0:
        drone.hover();
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        drone.move(leftr, forwardb, upd, turnleftr);
        cout << "00544!!!!!!!!" << " " << forwardb << endl;
        break;
      case 4:
      case 400:
      case 401:
      case 402:
      case 403:
      case 404:
      case 405:
        drone.move(leftr, forwardb, upd, 0.0);
        break;
      case 5:
        drone.move(leftr, forwardb, upd, 0.0);
        break;
      case 7:
        break;
      case 8:
        break;
      default:
        drone.hover();
      }
#endif
      break;
    }

    //cout << endl << "No. of frames: "<< frame_count <<endl;
  }
  drone.land();
  cvReleaseImage(&imgr);
  cvReleaseImage(&imgg);
  cvReleaseImage(&imgb);
  cvReleaseImage(&imgsrc);
  cvReleaseImage(&imgyellow);
  cvReleaseImage(&imgnumberwarp);
  cvReleaseImage(&squaretmp);
  cvReleaseImage(&squaretmp1);
  return 0;
}

void ROSControl_main(int argc, char** argv) {
  //ros::init(argc, argv, "listener");
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
int main(int argc, char** argv)
{
#if premode==1
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel1.xml");
#else
#if premode==2
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel2.xml");
#else
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel_op.xml");
#endif
#endif
  ros::init(argc, argv, "ARDrone_test");
  ROSControl_main(argc, argv);
  fout.close();

  return 0;
}
