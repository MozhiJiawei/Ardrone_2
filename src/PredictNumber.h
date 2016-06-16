#ifndef   __PREDICTNUMBER_H__ 
#define   __PREDICTNUMBER_H__ 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#define premode 1//1->size*size/4,2->size*size,3->opt
using namespace cv;

double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 );
void findNumberCenter(IplImage *imgsrc,double &centerxavr,double &centeryavr);
void EqualizeHistColorImage(IplImage *pImage);
int findones(IplImage* src,CvRect r);
void thinImage(IplImage* src,IplImage* dst,int maxIterations = -1 );
void cvThin( IplImage* src, IplImage* dst, int iterations/*=1*/);
float Color_Detection_Pro(IplImage* src,double &xpositon,double& yposition);
float Color_Detection_Pro(IplImage* src,double &xpositon,double& yposition,double& xpositionblue,double& ypositionblue,int mode);
IplImage* mycopy(IplImage* src,CvRect r);
IplImage *norm(IplImage* img);
IplImage *norm(IplImage* img,int n,int m);
IplImage* divide(IplImage* img,int threshould1,int threshould2);
void  divide(IplImage* img,CvRect r,CvRect& rec);
void domask(int flag,IplImage* pImg,CvSeq* contours);
void mdomask(IplImage* pImg,IplImage* pImg1);
Mat characteristic(IplImage* img,int flag,int size);
int predictNumber(IplImage* img,int size);

#endif
