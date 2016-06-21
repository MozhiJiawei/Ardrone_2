#include <cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char *argv[])
{
  //CvCapture* pCapture = cvCreateCameraCapture(1);
  //cvNamedWindow("Video", 1);

  cv::VideoCapture cap(1);
  cv::namedWindow("Video", 1);
  CV_ASSERT(cap.isOpened);
  cv::Mat frame;
  while (1)
  {
    //IplImage* pFrame=cvQueryFrame( pCapture );
    //if(!pFrame)break;
    //cvShowImage("Video",pFrame);
    cap >> frame;
    char c = cv::waitKey(3);
    if (c == 27)break;
  }
  return 0;
}
