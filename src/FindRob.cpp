// FindRob.cpp: implementation of the FindRob class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "FindRob.h"

//1-main switch for the following show many images for test,0-close the function
#define TestShowImg 0

//1-show robot threshold pic,0-close the function
#define ShowRobotT 1
//1-show ground threshold pic,0-close the function
#define ShowGroundT 1
//1-show ground center threshold pic,0-close the function
#define ShowGroundCenterT 1

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

FindRob::FindRob(IplImage *img)
{
  ImgForRob = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  ImgForBlue = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  ImgForYellow = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  OriImg = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 3);
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
  cvCopy( img, OriImg);
  OriginImg = img;
  
  //cvCopy(img, InputImg);
  RobotCenter.x = 320;
  RobotCenter.y = 180;
  RobotBlackPoint.x = 320;
  RobotBlackPoint.y = 500;
  GroundCenter = cvPoint(-100, -100);
  minX=640, maxX=0, minY=360, maxY=0;
  GroundRadius = 0;
  RobotRadius = 0;
  GroundDir = 0;

  isEdge = 0;
  isEdgeConfirm = 0;
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
        
      if((c-a)>80 && (c-b)>80)//for robotc>100 && b<70 && a<200
      {        
        q[i*ImgForRob->nChannels+(j)*ImgForRob->widthStep]=255;          
      }
      else
        /*
        if((a+b+c)>600)//let white circle be white
          q[i*ImgForRob->nChannels+(j)*ImgForRob->widthStep]=255;
        else
        */
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
  if(FlagSearRob != 0) return;

  cvErode(src,src,NULL,1);
  //cvDilate(src,src,NULL,4);
  //cvErode(src,src,NULL,1);
#if TestShowImg && ShowRobotT
  cvNamedWindow("robot threshold",0);
  cvShowImage("robot threshold", ImgForRob);
  cvMoveWindow("robot threshold", 600, 1);
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
  //int edge = isGroundEdge();

  CvRect recttemp, maxrect;
  maxrect.x = 0;
  maxrect.y = 0;
  maxrect.width  = 0;
  maxrect.height = 0;
  while(tempcont != NULL)
  {
    //cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);
    //use retangle to find the largest as robot
    recttemp = cvBoundingRect(tempcont, 0);
    //cout<<"radius of circle:"<<tempR<<endl;
    //if(tempR > RobRadius)
    if((recttemp.width * recttemp.height) > (maxrect.width * maxrect.height))
    {
      maxrect = recttemp;
      RobCont = tempcont;
      /*if(edge==0 || (edge!=0 && tempCenter.x>minX && tempCenter.x<maxX && tempCenter.y>minY && tempCenter.y<maxY))
      {
        RobCenter = tempCenter;
        RobRadius = tempR;
        RobCont = tempcont;
      }*/
    }
    tempcont = tempcont->h_next;
  }

  if(RobCont != NULL)
  {
    cvMinEnclosingCircle( RobCont, &RobCenter, &RobRadius);
  }


  if(RobRadius > 20)//avoid misregconize small points as robot
  {
  FlagRobotExist = 1;
  RobotCenter = cvPointFrom32f(RobCenter);
  RobotRadius = cvRound(RobRadius-5);
  cvCircle (OriginImg, RobotCenter, RobotRadius, CV_RGB(225, 250, 225), 3, 8, 0);

  //find the white circle in robot
  int a = 0, b = 0, c = 0;
  IplImage *imgwhite = cvCreateImage( cvSize(640, 360), IPL_DEPTH_8U, 1);
  cvZero(imgwhite);
  unsigned char* p = (unsigned char*)(OriImg->imageData);
  unsigned char* q = (unsigned char*)(imgwhite->imageData);
  for(int i=maxrect.x; i<maxrect.x+maxrect.width; ++i)
  {
		for(int j=maxrect.y; j<maxrect.y+maxrect.height; ++j)
		{
      a=(int)p[i*OriImg->nChannels+(j)*OriImg->widthStep];
      b=(int)p[i*OriImg->nChannels+(j)*OriImg->widthStep+1];
      c=(int)p[i*OriImg->nChannels+(j)*OriImg->widthStep+2];
      if((a+b+c) > 600)
        q[i*imgwhite->nChannels+(j)*imgwhite->widthStep]=255;
    }
  }
#if TestShowImg && ShowRobotT
  cvErode(imgwhite, imgwhite, NULL, 1);
  cvNamedWindow("robot white circle",0);
  cvShowImage("robot white circle", imgwhite);
  cvMoveWindow("robot white circle", 600, 600);
  cvWaitKey(1);
#endif
  CvMemStorage *storage1 = cvCreateMemStorage(0);
  CvSeq *contwhite = 0;
  int contourswhite = 0;
  contourswhite = cvFindContours( imgwhite, storage1, &contwhite, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //cout<<"cont white:"<<contourswhite<<endl;
  if(contourswhite > 0)
  {
    CvSeq * maxcontwhite = contwhite;
    CvPoint2D32f center1;
    float r1 = 0;
    for(tempcont = contwhite; tempcont != NULL; tempcont = tempcont->h_next)//find largest white circle
    {    
      cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);    
      if(tempR > r1)
      {
        center1 = tempCenter;
        r1 = tempR;
      }
      //cout<<"r1: "<<r1<<endl;
    }
    if(r1 > (RobRadius/5))
    {
      RobotBlackPoint = cvPointFrom32f(center1);
      //cout<<"white point:"<<RobotBlackPoint.x<<" "<<RobotBlackPoint.y<<", r:"<<r1<<endl;
      cvCircle (OriginImg, RobotBlackPoint, cvRound(r1), CV_RGB(25, 200, 255), 3, 8, 0);
    }

    cvReleaseImage(&imgwhite);
    //if(contwhite != NULL) 
    //{
      //cvClearSeq(contwhite);
      cvReleaseMemStorage( &storage1 );
    //}
  }

  /*
  if(RobCont->v_next != NULL && RobRadius > 50)
  {    
    CvPoint2D32f center1;
    float r1 = 0;
    for(tempcont = RobCont->v_next; tempcont !=NULL; tempcont = tempcont->h_next)//find largest circle
    {    
      cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);    
      if(tempR > r1)
      {
        center1 = tempCenter;
        r1 = tempR;
      }
    //cvCircle (img, cvPoint(cvRound(center1.x), cvRound(center1.y)), cvRound(r1), CV_RGB(0, 255, 127), 3, 8, 0);
    }
    if((r1 > (RobRadius/5)) && (sqrt( (center1.x-RobCenter.x)*(center1.x-RobCenter.x) + (center1.y-RobCenter.y)*(center1.y-RobCenter.y) ) > (RobRadius/4)))//judge whether is side black circle
    {
      RobotBlackPoint = cvPointFrom32f(center1);
      cvCircle (OriginImg, RobotBlackPoint, cvRound(r1), CV_RGB(25, 200, 255), 3, 8, 0);
    }
  }*/
    //
    /*if(RobCont->v_next->h_next != NULL)
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
    }*/      
  }
  }
  FlagSearRob = 1;
  
  //if(contour!=NULL) cvClearSeq(contour);
  //{
    //
  //cout<<"analyze red done!\n";
    cvReleaseMemStorage( &storage );
  //}
}

//analyze yellow
void FindRob::AnalyzeGround(IplImage *src)
{
  if(FlagAnaGrou != 0) return;

  cvErode(src,src,NULL,1);
#if TestShowImg && ShowGroundT
  cvNamedWindow("ground yellow threshold",0);
  cvShowImage("ground yellow threshold", ImgForYellow);
  cvMoveWindow("ground yellow threshold", 60, 350);
  cvWaitKey(1);
#endif
  CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
  CvSeq *contour = 0, *RobCont = 0, *tempcont = 0;
  int contours = 0;
  contours = cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  //find min bounding rect, and find maxcontour
  CvRect recttemp;
  int tempx=0, tempy=0;
  int maxArea = 0, tempArea = 0;
  CvSeq *maxcont = 0, *cont = 0;
  for(tempcont = contour; tempcont!=NULL; tempcont = tempcont->h_next)
  {
    recttemp = cvBoundingRect(tempcont, 0);
    if (recttemp.width > 20 || recttemp.height > 20)  
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

    //find maxcont
    tempArea = fabs(cvContourArea(tempcont));
    if(tempArea > maxArea)
    {
      maxcont = tempcont;
      maxArea = tempArea;
    }
  }//cout<<"maxXminXmaxYminY:"<<maxX<<" "<<minX<<" "<<maxY<<" "<<minY<<endl;
  if(minX > 5)//left
    isEdge += 2;
  if(minY > 5)//forward
    isEdge += 8;
  if(maxX < 635)//right
    isEdge += 1;
  if(maxY < 355)//back
    isEdge += 4;

  switch(isEdge)
  {
    case 1:{maxX -= 10;minX -= 300;maxY += 300;minY -= 300;}break;
    case 2:{minX += 10;maxX += 300;maxY += 300;minY -= 300;}break;
    case 4:{maxY -= 10;minY -= 300;maxX += 300;minX -= 300;}break;
    case 8:{minY += 10;maxY += 300;maxX += 300;minX -= 300;}break;
    case 9:{minY += 10;maxX -= 10;maxY += 300;minX -= 300;}break;
    case 10:{minY += 10;minX += 10;maxY += 300;maxX += 300;}break;
    case 5:{maxY -= 10;maxX -= 10;minY -= 300;minX -= 300;}break;
    case 6:{maxY -= 10;minX += 10;minY -= 300;maxX += 300;}break;
  }
//try to confirm the ground angle
  if(maxArea > 1000)
  {
    //ni he zhi xian
    cont = cvApproxPoly(maxcont, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcont)*0.005, 0);
    cvDrawContours(OriginImg, cont, CV_RGB(225, 220, 25), CV_RGB(25, 20, 255), 0, 2);
    
    bool flagpredis = 0, flagnextdis = 0;
    for(int i=0; i<cont->total; ++i)
    {      
      CvPoint *pt0 = (CvPoint*)cvGetSeqElem( cont, i-1 ),
        *pt1 = (CvPoint*)cvGetSeqElem( cont, i ),
        *pt2 = (CvPoint*)cvGetSeqElem( cont, i-2 );
      if(ptdistance(pt0, pt1) > 60 && ((pt0->x >10 && pt0->x <630 && pt0->y >10 && pt0->y <350) || (pt1->x >10 && pt1->x <630 && pt1->y >10 && pt1->y <350))) 
        flagnextdis = 1;

      if(flagpredis && flagnextdis)
      {
        if(fabs(angle(pt1, pt2, pt0)) < 0.17)//is 90 degree(+-10)
        {          
          cvCircle (OriginImg, *pt0, 10, CV_RGB(125, 100, 255), 2, 8, 0);
          double dx1 = pt1->x - pt0->x, dy1 = pt1->y - pt0->y;
          if(fabs(dx1) > fabs(dy1))
          {
            if(dx1 < 0)
            {
              GroundDir = asin(dy1 / sqrt(dx1*dx1 + dy1*dy1));
            }
            else
            {
              GroundDir = -asin(dy1 / sqrt(dx1*dx1 + dy1*dy1));
            }
          }
          else
          {
            dx1 = pt2->x - pt0->x;
            dy1 = pt2->y - pt0->y;
            if(dx1 < 0)
            {
              GroundDir = asin(dy1 / sqrt(dx1*dx1 + dy1*dy1));
            }
            else
            {
              GroundDir = -asin(dy1 / sqrt(dx1*dx1 + dy1*dy1));
            }
          }          
          break;
        }
      }
      flagpredis = flagnextdis;
      flagnextdis = 0;
    }
    //if(cont!=NULL) cvClearSeq(cont);
    cvReleaseMemStorage( &storage1 );
  }
//cout<<"after-maxXminXmaxYminY:"<<maxX<<" "<<minX<<" "<<maxY<<" "<<minY<<endl;
  FlagAnaGrou = 1;
  //cout<<"analyze yellow done!\n";
  //if(contour!=NULL) cvClearSeq(contour);  
  cvReleaseMemStorage( &storage );  
}

//analyze blue
void FindRob::FindGroundCenter(IplImage *src)
{
  if(FlagFindGrCenter != 0) return;
  if(FlagAnaGrou == 0)
    AnalyzeGround(ImgForYellow);

  if(FlagAnaGrou == 0)
    AnalyzeGround(ImgForYellow);
  cvErode(src,src,NULL,2);
#if TestShowImg && ShowGroundCenterT
  cvNamedWindow("ground center blue threshold",0);
  cvShowImage("ground center blue threshold", ImgForBlue);
  cvMoveWindow("ground center blue threshold", 60, 1);
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
    int edge = isEdge;//isGroundEdge(),mistake here
    while(tempcont != NULL)
    {
      cvMinEnclosingCircle( tempcont, &tempCenter, &tempR);
      if(tempR > RobRadius)
      { 
        if(edge==0 || (edge!=0 && tempCenter.x>(minX+100) && tempCenter.x<(maxX-100) && tempCenter.y>(minY+100) && tempCenter.y<(maxY-100)))
        {
        RobCenter = tempCenter;
        RobRadius = tempR;
        RobCont = tempcont;  
        }        
      }

      if(tempR > 20)//judge whether the blue is edge
      {
        if((8 & isEdge) && tempCenter.y<(minY+100))
        {
          isEdgeConfirm += 8;
        }
        if((4 & isEdge) && tempCenter.y>(maxY-100))
        {
          isEdgeConfirm += 4;
        }
        if((2 & isEdge) && tempCenter.x<(minX+100))
        {
          isEdgeConfirm += 2;
        }
        if((1 & isEdge) && tempCenter.x>(maxX-100))
        {
          isEdgeConfirm += 1;
        }
      }
      tempcont = tempcont->h_next;
    }
      if(RobRadius > 30)//avoid misregconize small points as robot
      {
        FlagGroundCenterExist = 1;
        GroundCenter = cvPointFrom32f(RobCenter);
        GroundRadius = cvRound(RobRadius);
#if TestShowImg && ShowGroundCenterT
  cvCircle (OriginImg, GroundCenter, GroundRadius, CV_RGB(25, 250, 25), 2, 8, 0);
#endif
      }    
  }
  FlagFindGrCenter = 1;
  //cout<<"analyze blue done!\n";
  //cvClearSeq(contour);
  cvReleaseMemStorage( &storage );
}

//for yellow
int FindRob::isGroundEdge()
{
  if(FlagAnaGrou == 0)
    AnalyzeGround(ImgForYellow);
  if(FlagFindGrCenter == 0)
    FindGroundCenter(ImgForBlue);

  return isEdgeConfirm;
}

double FindRob::getGroundDir()
{
  if(FlagAnaGrou == 0)
    AnalyzeGround(ImgForYellow);

  return GroundDir;
}

//for blue
CvPoint FindRob::getGroundCenter()
{  
  if(FlagFindGrCenter == 0)
    FindGroundCenter(ImgForBlue);

  return GroundCenter;
}

int FindRob::getGroundCenterRadius()
{
  if(FlagFindGrCenter == 0)
    FindGroundCenter(ImgForBlue);

  return GroundRadius;
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

int FindRob::getRobRadius()
{
  if(FlagSearRob == 0)
    SearchRobot(ImgForRob);

  return RobotRadius;
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

  double ang = - asin((double)(RobotCenter.x-RobotBlackPoint.x) /
    sqrt((RobotCenter.x-RobotBlackPoint.x) * (RobotCenter.x-RobotBlackPoint.x) + 
    (RobotBlackPoint.y-RobotCenter.y) * (RobotBlackPoint.y-RobotCenter.y)));

  if(RobotBlackPoint.y > RobotCenter.y)
  {
    if(ang > 0)
      ang = 3.1415 - ang;
    else 
      ang = -3.1415 - ang;
  }
  return ang;
}

//
double FindRob::angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2));// + 1e-10
}

double FindRob::ptdistance(CvPoint* pt1, CvPoint* pt2)
{
  double dx1 = pt1->x - pt2->x;
  double dy1 = pt1->y - pt2->y;
  return sqrt(dx1*dx1 + dy1*dy1);
}