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
  if(!cap.isOpened()) {
    std::cout << "Cannot Open Camera!" << std::endl; 
    return 0;
  }
  cv::namedWindow("Video", 1);
  cv::Mat frame;
  while (1)
  {
    //IplImage* pFrame=cvQueryFrame( pCapture );
    //if(!pFrame)break;
    //cvShowImage("Video",pFrame);
    cap >> frame;
    cv::imshow("Video", frame);
    char c = cv::waitKey(3);
    if (c == 27)break;
  }
  return 0;
}
