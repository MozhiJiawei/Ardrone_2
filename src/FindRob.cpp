// FindRob.cpp: implementation of the FindRob class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "FindRob.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

FindRob::FindRob(IplImage *img)
{
  ImgForRob = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  ImgForBlue = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  ImgForYellow = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  if(img != NULL)
  {
    ReInit(img);
  }
}

FindRob::~FindRob()
{

}

void FindRob::ReInit(IplImage *img)
{
  OriginImg = img;
  
  //cvCopy(img, InputImg);
  RobotCenter.x = 320;
  RobotCenter.y = 180;
  RobotBlackPoint.x = 320;
  RobotBlackPoint.y = -100;
  GroundCenter = cvPoint(-100, -100);

  isEdge = 0;
  FlagRobotExist = 0;
  FlagGroundCenterExist = 0;
  FlagSearRob = 0;
  FlagAnaGrou = 0;
  FlagFindGrCenter = 0;

  //threshold,a-blue,b-green,c-red,red and white circle is white,
  int a = 0, b = 0, c = 0;
  unsigned char* p = (unsigned char*)(img->imageData);
  unsigned char* q = (unsigned char*)(ImgForRob->imageData);
  unsigned char* r = (unsigned char*)(ImgForYellow->imageData);
  unsigned char* s = (unsigned char*)(ImgForBlue->imageData);
  int i = 0, j = 0;
  for(i=0;i<img->width;++i)
  {
		for(j=0;j<img->height;++j)
		{
      a=(int)p[i*img->nChannels+(j)*img->widthStep];
      b=(int)p[i*img->nChannels+(j)*img->widthStep+1];
      c=(int)p[i*img->nChannels+(j)*img->widthStep+2];
        
      if(c>100 && b<70 && a<200)//for robot
      {        
        q[i*ImgForRob->nChannels+(j)*ImgForRob->widthStep]=255;          
      }
      else
        if((a+b+c)>600)//let white circle be white
          q[i*ImgForRob->nChannels+(j)*ImgForRob->widthStep]=255;
        else
          q[i*ImgForRob->nChannels+(j)*ImgForRob->widthStep]=0;

      if((c-a)>0 && (b-a)>0)//for yellow
      {
        if((a+b+c)<50)
          r[i*ImgForYellow->nChannels+(j)*ImgForYellow->widthStep]=0;
        else
          r[i*ImgForYellow->nChannels+(j)*ImgForYellow->widthStep]=255;          
      }
      else
        r[i*ImgForYellow->nChannels+(j)*ImgForYellow->widthStep]=0;

      if(b<20 && c<20 && a>50)//for blue
      {          
        s[i*ImgForBlue->nChannels+(j)*ImgForBlue->widthStep]=255;          
      }
      else
        s[i*ImgForBlue->nChannels+(j)*ImgForBlue->widthStep]=0;
    }
  }
}

//analyze red
void FindRob::SearchRobot(IplImage *src)
{  
  cvErode(src,src,NULL,1);
  cvDilate(src,src,NULL,4);
  cvErode(src,src,NULL,1);
#if TestShowImg && ShowRobotT
  cvNamedWindow("robot threshold",0);
  cvShowImage("robot threshold", ImgForRob);
  cvWaitKey(1);
#endif
  //find contours
  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *contour = 0, *RobCont = 0, *tempcont = 0;
  int contours = 0;
  contours = cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //cout<<"number of contours:"<<contours<<endl;
  if(contours > 0)
  {
  //choose largest circle as the robot
  tempcont = contour;
  RobCont = contour;
  //double face;
  CvPoint2D32f RobCenter, tempCenter;
  float RobRadius = 0, tempR = 0;
  while(tempcont != NULL)
  {
    cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);
    if(tempR > RobRadius)
    {
      //face = 3.14159*tempR*tempR;
      //if((face-fabs(cvContourArea(tempcont))) < 0.3*face)
      //{
        RobCenter = tempCenter;
        RobRadius = tempR;
        RobCont = tempcont;
      //}
    }
    tempcont = tempcont->h_next;
  }
  if(RobRadius > 30)//avoid misregconize small points as robot
  {
  FlagRobotExist = 1;
  RobotCenter = cvPointFrom32f(RobCenter);
  cvCircle (OriginImg, RobotCenter, cvRound(RobRadius-10), CV_RGB(225, 250, 225), 3, 8, 0);
  if(RobCont->v_next != NULL && RobRadius > 50)
  {
    CvPoint2D32f center1;
    float r1;
    cvMinEnclosingCircle( RobCont->v_next, &center1, &r1);    
    //cvCircle (img, cvPoint(cvRound(center1.x), cvRound(center1.y)), cvRound(r1), CV_RGB(0, 255, 127), 3, 8, 0);

    if(RobCont->v_next->h_next != NULL)
    {
      CvPoint2D32f center2;
      float r2;
      cvMinEnclosingCircle( RobCont->v_next->h_next, &center2, &r2);      
      //cvCircle (img, cvPoint(cvRound(center2.x), cvRound(center2.y)), cvRound(r2), CV_RGB(25, 200, 255), 3, 8, 0);
      if(r1 > r2)
      {
        RobotBlackPoint = cvPointFrom32f(center1);
      }
      else
      {
        RobotBlackPoint = cvPointFrom32f(center2);
        r1 = r2;
      }
      cvCircle (OriginImg, RobotBlackPoint, cvRound(r1), CV_RGB(25, 200, 255), 3, 8, 0);
    }    
  }
  }
  }
  FlagSearRob = 1;
  cvReleaseMemStorage( &storage );
}

//analyze yellow
void FindRob::AnalyzeGround(IplImage *src)
{
  cvErode(src,src,NULL,1);
#if TestShowImg && ShowGroundT
  cvNamedWindow("ground yellow threshold",0);
  cvShowImage("ground yellow threshold", ImgForYellow);
  cvWaitKey(1);
#endif
  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *contour = 0, *RobCont = 0, *tempcont = 0;
  int contours = 0;
  contours = cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  //find min bounding rect
  CvRect recttemp;
  int minX=640, maxX=0, minY=360, maxY=0, tempx=0, tempy=0;
  for(tempcont = contour; tempcont!=NULL; tempcont = tempcont->h_next)
  {
    recttemp = cvBoundingRect(tempcont, 0);
    if (recttemp.width > 20 && recttemp.height > 20)  
    {
      tempx = recttemp.x;
      if(tempx < minX)
        minX = tempx;

      tempx = recttemp.x + recttemp.width;
      if(tempx > maxX)
        maxX = tempx;

      tempy = recttemp.y;
      if(tempy < minY)
        minY = tempy;

      tempy = recttemp.y + recttemp.height;
      if(tempy > maxY)
        maxY = tempy;      
    }
  }
  if(minX > 5)//left
    isEdge += 2;
  if(minY > 5)//forward
    isEdge += 8;
  if(maxX < 635)//right
    isEdge += 1;
  if(maxY < 355)//back
    isEdge += 4;

  FlagAnaGrou = 1;
  cvReleaseMemStorage( &storage );
}

//analyze blue
void FindRob::FindGroundCenter(IplImage *src)
{
  cvErode(src,src,NULL,2);
#if TestShowImg && ShowGroundCenterT
  cvNamedWindow("ground center blue threshold",0);
  cvShowImage("ground center blue threshold", ImgForBlue);
  cvWaitKey(1);
#endif

  //here reuse find robot contour
  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *contour = 0, *RobCont = 0, *tempcont = 0;
  int contours = 0;
  contours = cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  if(contours > 0)
  {
    //choose largest circle as the ground center
    tempcont = contour;
    RobCont = contour;
    //double face;
    CvPoint2D32f RobCenter, tempCenter;
    float RobRadius = 0, tempR = 0;
    while(tempcont != NULL)
    {
      cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);
      if(tempR > RobRadius)
      {      
        RobCenter = tempCenter;
        RobRadius = tempR;
        RobCont = tempcont;      
      }
      tempcont = tempcont->h_next;
  
      if(RobRadius > 30)//avoid misregconize small points as robot
      {
        FlagGroundCenterExist = 1;
        GroundCenter = cvPointFrom32f(RobCenter);
#if TestShowImg && ShowGroundCenterT
  cvCircle (OriginImg, GroundCenter, cvRound(RobRadius-5), CV_RGB(25, 250, 25), 2, 8, 0);
#endif
      }
    }
  }
  FlagFindGrCenter = 1;
  cvReleaseMemStorage( &storage );
}

//for yellow
int FindRob::isGroundEdge()
{
  if(FlagAnaGrou == 0)
    AnalyzeGround(ImgForYellow);

  return isEdge;
}

//for blue
CvPoint FindRob::getGroundCenter()
{
  if(FlagFindGrCenter == 0)
    FindGroundCenter(ImgForBlue);

  return GroundCenter;
}

bool FindRob::doesGroundCenterExist()
{
  if(FlagFindGrCenter == 0)
    FindGroundCenter(ImgForBlue);

  return FlagGroundCenterExist;
}

//for robot
CvPoint FindRob::getRobCenter()
{
  if(FlagSearRob == 0)
    SearchRobot(ImgForRob);

  return RobotCenter;
}

bool FindRob::doesRobotExist()
{
  if(FlagSearRob == 0)
    SearchRobot(ImgForRob);

  return FlagRobotExist;
}

double FindRob::getRobDir()
{
  if(FlagSearRob == 0)
    SearchRobot(ImgForRob);

  double ang = asin((double)(RobotCenter.x-RobotBlackPoint.x) /
    sqrt((RobotCenter.x-RobotBlackPoint.x) * (RobotCenter.x-RobotBlackPoint.x) + 
    (RobotBlackPoint.y-RobotCenter.y) * (RobotBlackPoint.y-RobotCenter.y)));

  if(RobotBlackPoint.y < RobotCenter.y)
  {
    if(ang > 0)
      ang = 3.1415 - ang;
    else 
      ang = -3.1415 - ang;
  }
  return ang;
}