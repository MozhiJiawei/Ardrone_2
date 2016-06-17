// FindRob.h: interface for the FindRob class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_FINDROB_H__72A23126_B93C_4525_9F91_B5C7F379DC83__INCLUDED_)
#define AFX_FINDROB_H__72A23126_B93C_4525_9F91_B5C7F379DC83__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include <iostream>
#include <string>
#include <math.h>
#include <highgui.h>
#include <cv.h>

using namespace std;

class FindRob  
{
public:
  //Attention:you can't do any changes to the img outside until you don't use this class
	FindRob(IplImage *img);//input picture, NULL is ok
	virtual ~FindRob();
  void ReInit(IplImage *img);//change picture, NULL is not allowed!

  //about robot
  CvPoint getRobCenter();//get the center of robot
  double getRobDir();//get robot direction(asin), forward is 0, left is -pai~0, right is 0~+pai

  //about ground
  int isGroundEdge();//does edge appear in the camera?0-no, 8-forward, 4-back, 2-left, 1-right, 10-forward left, 9-forward right, 6-back left, 5-back right
  CvPoint getGroundCenter();//return blue point center,if not exist, return (-100,-100)

private:
  IplImage *OriginImg,
    *ImgForRob,
    *ImgForYellow,
    *ImgForBlue;
  CvPoint RobotCenter;
  CvPoint RobotBlackPoint;
  CvPoint GroundCenter;
  int isEdge;

  void SearchRobot(IplImage *src);
  int FlagSearRob;
  void AnalyzeGround(IplImage *src);
  int FlagAnaGrou;
};

#endif // !defined(AFX_FINDROB_H__72A23126_B93C_4525_9F91_B5C7F379DC83__INCLUDED_)
