#ifndef   __AFFINETRANSFORM_H__ 
#define   __AFFINETRANSFORM_H__ 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <opencv2/opencv.hpp>
using namespace cv;

static double angle( Point pt1, Point pt2, Point pt0);
void findSquares( const Mat& image, vector<vector<Point> >& squares,int mode);
void drawSquares( Mat& image, const vector<vector<Point> >& squares );
double GetYawFromFrontCam(vector<vector<Point> > squares,float scale);
CvMat *warpPerspective(IplImage *src, IplImage *dst,vector<Point> Top);
static bool VectorPointGreat(const Point &pt1,const Point &pt2)
{
  //构造一个能顺时针比较点大小的判断
  if((pt1.x>pt2.x)&&(pt1.y<pt2.y)&&(abs(pt1.x-pt2.x)<5*abs(pt1.y-pt2.y)))
  {
    return pt1.y<pt2.y;
  }
  else
  {
    return (pt1.x+pt1.y)<(pt2.x+pt2.y);
  }
}

#endif
