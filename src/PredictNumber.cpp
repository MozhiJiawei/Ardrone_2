#include "PredictNumber.h"
//函数：color_detection_pro
//功能：对图片提取颜色滤波只保留需要的颜色
//          留下黄色
using namespace std;
extern CvANN_MLP bp;
void findNumberCenter(IplImage *imgsrc,double &centerxavr,double &centeryavr)
{
  CvRect r,rec;
  CvSize sz;
  r.x=40,r.y=40,r.height=240,r.width=600;
  sz.height=r.height,sz.width=r.width;
  IplImage *imgtmp = cvCreateImage(sz,IPL_DEPTH_8U,1);
  cvSetImageROI(imgsrc,r);
  cvCvtColor(imgsrc,imgtmp,CV_BGR2GRAY); //转换
  cvThreshold(imgtmp, imgtmp, 40, 1, CV_THRESH_BINARY_INV);
  r.height=0,r.width=0,r.x=0,r.y=0;
  divide(imgtmp,r,rec);
  cvResetImageROI(imgsrc);
  centerxavr=rec.x+rec.width/2;
  centeryavr=rec.y+rec.height/2;
  cvReleaseImage(&imgtmp);
}

void EqualizeHistColorImage(IplImage *pImage)
{

  // 原图像分成各通道后再均衡化,最后合并即彩色图像的直方图均衡化
  const int MAX_CHANNEL = 4;
  IplImage *pImageChannel[MAX_CHANNEL] = {NULL};

  int i;
  for (i = 0; i < pImage->nChannels; i++)
    pImageChannel[i] = cvCreateImage(cvGetSize(pImage), pImage->depth, 1);

  cvSplit(pImage, pImageChannel[0], pImageChannel[1], pImageChannel[2], pImageChannel[3]);

  for (i = 0; i < pImage->nChannels; i++)
    cvEqualizeHist(pImageChannel[i], pImageChannel[i]);

  cvMerge(pImageChannel[0], pImageChannel[1], pImageChannel[2], pImageChannel[3], pImage);

  for (i = 0; i < pImage->nChannels; i++)
    cvReleaseImage(&pImageChannel[i]);
}
int findones(IplImage* src,CvRect r)
{
  int counts=0;
  for(int i=r.y;i<=r.y+r.height;++i)
  {
    for(int j=r.x*src->nChannels;j<=(r.x+r.width)*src->nChannels;j+=3)
    {
      if(((src->imageData)[i*(src->widthStep)+j]&0x00ff)==255)
      {
        ++counts;
      }
    }
  }
  return 3*counts;
}
void thinImage(IplImage* src,IplImage* dst,int maxIterations )
{
  CvSize size = cvGetSize(src);
  cvCopy(src,dst);//将src中的内容拷贝到dst中
  int count = 0;  //记录迭代次数
  while (true)
  {
    count++;
    if(maxIterations!=-1 && count > maxIterations) //限制次数并且迭代次数到达
      break;
    //std::cout << count << ' ';输出迭代次数
    vector<pair<int,int> > mFlag; //用于标记需要删除的点
    //对点标记
    for (int i=0; i<size.height; ++i)
    {
      for (int j=0; j<size.width; ++j)
      {
        //如果满足四个条件，进行标记
        //  p9 p2 p3
        //  p8 p1 p4
        //  p7 p6 p5
        int p1 = CV_IMAGE_ELEM(dst,uchar,i,j);
        int p2 = (i==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j);
        int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j+1);
        int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i,j+1);
        int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j+1);
        int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j);
        int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j-1);
        int p8 = (j==0)?0:CV_IMAGE_ELEM(dst,uchar,i,j-1);
        int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j-1);

        if ((p2+p3+p4+p5+p6+p7+p8+p9)>=2 && (p2+p3+p4+p5+p6+p7+p8+p9)<=6)
        {
          int ap=0;
          if (p2==0 && p3==1) ++ap;
          if (p3==0 && p4==1) ++ap;
          if (p4==0 && p5==1) ++ap;
          if (p5==0 && p6==1) ++ap;
          if (p6==0 && p7==1) ++ap;
          if (p7==0 && p8==1) ++ap;
          if (p8==0 && p9==1) ++ap;
          if (p9==0 && p2==1) ++ap;

          if (ap==1)
          {
            if (p2*p4*p6==0)
            {
              if (p4*p6*p8==0)
              {
                //标记
                mFlag.push_back(make_pair(i,j));
              }
            }
          }
        }
      }
    }

    //将标记的点删除
    for (vector<pair<int,int> >::iterator i=mFlag.begin(); i!=mFlag.end(); ++i)
    {
      CV_IMAGE_ELEM(dst,uchar,i->first,i->second) = 0;
    }

    //直到没有点满足，算法结束
    if (mFlag.size()==0)
    {
      break;
    }
    else
    {
      mFlag.clear();//将mFlag清空
    }

    //对点标记
    for (int i=0; i<size.height; ++i)
    {
      for (int j=0; j<size.width; ++j)
      {
        //如果满足四个条件，进行标记
        //  p9 p2 p3
        //  p8 p1 p4
        //  p7 p6 p5
        int p1 = CV_IMAGE_ELEM(dst,uchar,i,j);
        if(p1!=1) continue;
        int p2 = (i==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j);
        int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j+1);
        int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i,j+1);
        int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j+1);
        int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j);
        int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i+1,j-1);
        int p8 = (j==0)?0:CV_IMAGE_ELEM(dst,uchar,i,j-1);
        int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(dst,uchar,i-1,j-1);

        if ((p2+p3+p4+p5+p6+p7+p8+p9)>=2 && (p2+p3+p4+p5+p6+p7+p8+p9)<=6)
        {
          int ap=0;
          if (p2==0 && p3==1) ++ap;
          if (p3==0 && p4==1) ++ap;
          if (p4==0 && p5==1) ++ap;
          if (p5==0 && p6==1) ++ap;
          if (p6==0 && p7==1) ++ap;
          if (p7==0 && p8==1) ++ap;
          if (p8==0 && p9==1) ++ap;
          if (p9==0 && p2==1) ++ap;

          if (ap==1)
          {
            if (p2*p4*p8==0)
            {
              if (p2*p6*p8==0)
              {
                //标记
                mFlag.push_back(make_pair(i,j));
              }
            }
          }
        }
      }
    }
    //删除
    for (vector<pair<int,int> >::iterator i=mFlag.begin(); i!=mFlag.end(); ++i)
    {
      CV_IMAGE_ELEM(dst,uchar,i->first,i->second) = 0;
    }

    //直到没有点满足，算法结束
    if (mFlag.size()==0)
    {
      break;
    }
    else
    {
      mFlag.clear();//将mFlag清空
    }
  }
}
void cvThin( IplImage* src, IplImage* dst, int iterations/*=1*/)
{
  CvSize size = cvGetSize(src);
  cvCopy(src, dst);
  int n = 0,i = 0,j = 0;
  for(n=0; n<iterations; n++)
  {
    IplImage* t_image = cvCloneImage(dst);
    for(i=0; i<size.height;  i++)
    {
      for(j=0; j<size.width; j++)
      {
        if(CV_IMAGE_ELEM(t_image,char,i,j)!=0)
        {
          int ap=0;
          int p2 = (i==0)?0:CV_IMAGE_ELEM(t_image,char, i-1, j);
          int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char, i-1, j+1);
          if (p2==0 && p3!=0)
          {
            ap++;
          }
          int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char,i,j+1);
          if(p3==0 && p4!=0)
          {
            ap++;
          }
          int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char,i+1,j+1);
          if(p4==0 && p5!=0)
          {
            ap++;
          }
          int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(t_image,char,i+1,j);
          if(p5==0 && p6!=0)
          {
            ap++;
          }
          int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(t_image,char,i+1,j-1);
          if(p6==0 && p7!=0)
          {
            ap++;
          }
          int p8 = (j==0)?0:CV_IMAGE_ELEM(t_image,char,i,j-1);
          if(p7==0 && p8!=0)
          {
            ap++;
          }
          int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(t_image,char,i-1,j-1);
          if(p8==0 && p9!=0)
          {
            ap++;
          }
          if(p9==0 && p2!=0)
          {
            ap++;
          }
          if((p2+p3+p4+p5+p6+p7+p8+p9)>1 /*&& (p2+p3+p4+p5+p6+p7+p8+p9)<7*/)
          {
            if(ap!=0)
            {
              if(!(p2 && p4 && p6))
              {
                if(!(p4 && p6 && p8))
                {
                  CV_IMAGE_ELEM(dst,char,i,j)=0;
                }
              }
            }
          }

        }
      }
    }
    cvReleaseImage(&t_image);
    t_image = cvCloneImage(dst);
    for(i=0; i<size.height;  i++)
    {
      for(int j=0; j<size.width; j++)
      {
        if(CV_IMAGE_ELEM(t_image,char,i,j)!=0)
        {
          int ap=0;
          int p2 = (i==0)?0:CV_IMAGE_ELEM(t_image,char, i-1, j);
          int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char, i-1, j+1);
          if (p2==0 && p3!=0)
          {
            ap++;
          }
          int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char,i,j+1);
          if(p3==0 && p4!=0)
          {
            ap++;
          }
          int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,char,i+1,j+1);
          if(p4==0 && p5!=0)
          {
            ap++;
          }
          int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(t_image,char,i+1,j);
          if(p5==0 && p6!=0)
          {
            ap++;
          }
          int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(t_image,char,i+1,j-1);
          if(p6==0 && p7!=0)
          {
            ap++;
          }
          int p8 = (j==0)?0:CV_IMAGE_ELEM(t_image,char,i,j-1);
          if(p7==0 && p8!=0)
          {
            ap++;
          }
          int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(t_image,char,i-1,j-1);
          if(p8==0 && p9!=0)
          {
            ap++;
          }
          if(p9==0 && p2!=0)
          {
            ap++;
          }
          if((p2+p3+p4+p5+p6+p7+p8+p9)>1/* && (p2+p3+p4+p5+p6+p7+p8+p9)<7*/)
          {
            if(ap!=0)
            {
              if(p2*p4*p8==0)
              {
                if(p2*p6*p8==0)
                {
                  CV_IMAGE_ELEM(dst, char,i,j)=0;
                }
              }
            }
          }
        }

      }

    }
    cvReleaseImage(&t_image);
  }
}
float Color_Detection_Pro(IplImage* src,double &xpositon,double& yposition,double& xpositionblue,double& ypositionblue,int mode)
{
  int scale =10;
  int sum=0,xsum=0,ysum=0;
  int sumblue=0,xbluesum=0,ybluesum=0;
  float percent=0;
  CvSize size=cvGetSize(src);
  CvSize newsize;
  newsize.width=size.width/scale;
  newsize.height=size.height/scale;
  cvSmooth(src,src,CV_MEDIAN,5,5);
  /*for(int i = 0; i < src->height; i++)
    {
    for(int j = 0; j < src->width; j++)
    {
  //if(src->imageData[i*(src->widthStep)+j*(src->nChannels)]>10) src->imageData[i*(src->widthStep)+j*(src->nChannels)]-=10;
  //else src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;

  }
  }*/
  IplImage *src_float = cvCreateImage(size,IPL_DEPTH_32F, 3);
  cvConvertScale(src, src_float, 1.0, 0.0);
  IplImage *hsv_img = cvCreateImage(size, IPL_DEPTH_32F , 3);
  cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
  if(mode==3)
  {
    int step = hsv_img->widthStep/sizeof(float);
    int channels = hsv_img->nChannels;
    float * datafloat = (float *)hsv_img->imageData;
    for(int i = 0; i < hsv_img->height; i++)
    {
      for(int j = 0; j < hsv_img->width; j++)
      {
        if(datafloat[i*step + j*channels + 2]>40&&(datafloat[i*step + j*channels + 1]>0.3)&&((datafloat[i*step + j*channels]>=0&&datafloat[i*step + j*channels]<10)||(datafloat[i*step + j*channels]>340&&datafloat[i*step + j*channels]<=360)))
        {
          src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
          sum++;
          xsum+=(j+1);
          ysum+=(i+1);
        }
        else
        {
          src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
        }
      }
    }
    percent=1.0*sum/(hsv_img->height*hsv_img->width);
    if(sum>0)
    {
      xpositon=1.0*xsum/sum;
      yposition=1.0*ysum/sum;
    }
    else
    {
      xpositon=0;
      yposition=0;
    }
  }
  else
  {
    IplImage *src_blue = cvCreateImage(size,IPL_DEPTH_8U, 1);
    IplImage *src_blue_resize = cvCreateImage(newsize,IPL_DEPTH_8U, 1);
    IplImage *src_blue_canny = cvCreateImage(newsize,IPL_DEPTH_8U, 1);
    int step = hsv_img->widthStep/sizeof(float);
    int channels = hsv_img->nChannels;
    float * datafloat = (float *)hsv_img->imageData;
    for(int i = 0; i < hsv_img->height; i++)
    {
      for(int j = 0; j < hsv_img->width; j++)
      {
        if(datafloat[i*step + j*channels + 2]>40&&(datafloat[i*step + j*channels + 1]>0.5)&&(datafloat[i*step + j*channels]>40&&datafloat[i*step + j*channels]<80))
        {
          src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
          sum++;
          xsum+=(j+1);
          ysum+=(i+1);
        }
        else
        {
          src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
          src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
        }
        if(datafloat[i*step + j*channels + 2]>20&&(datafloat[i*step + j*channels + 1]>0.3)&&(datafloat[i*step + j*channels]>40&&datafloat[i*step + j*channels]<80))
        {
          src_blue->imageData[i*(src_blue->widthStep)+j*(src_blue->nChannels)]=255;
          sumblue++;
          xbluesum+=(j+1);
          ybluesum+=(i+1);
        }
        else
        {
          src_blue->imageData[i*(src_blue->widthStep)+j*(src_blue->nChannels)]=0;
        }
      }
    }
    //detect lines in blue mode
    cvResize(src_blue,src_blue_resize);
    cvCanny(src_blue_resize,src_blue_canny,200,220,3);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;
    lines = cvHoughLines2(src_blue_canny, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 30, 10);
    if(lines->total>0)
    {

      CvPoint* line = (CvPoint*)cvGetSeqElem(lines, 0);
      if((1.0*abs(line[0].y-line[1].y)/(abs(line[0].x-line[1].x)+0.1))>newsize.height/10)
      {
        xpositionblue=(line[0].x+line[1].x)*scale/2;
        ypositionblue=(line[0].x+line[1].x)*scale/2;
        cout<<"xposition"<<xpositionblue<<"yposition"<<ypositionblue<<endl;
        cvLine(src_blue_canny, line[0], line[1], CV_RGB(255, 255, 255), 3, CV_AA, 0);
      }
      else
      {
        xpositionblue=0;
        ypositionblue=0;
      }

    }
    cvShowImage("line Detector",src_blue_canny);
    percent=1.0*sum/(hsv_img->height*hsv_img->width);
    if(sum>0)
    {
      xpositon=1.0*xsum/sum;
      yposition=1.0*ysum/sum;
    }
    else
    {
      xpositon=0;
      yposition=0;
    }
    cvReleaseImage(&src_blue);
    cvReleaseImage(&src_blue_resize);
    cvReleaseImage(&src_blue_canny);
    cvReleaseMemStorage(&storage);
  }
  cvReleaseImage(&hsv_img);
  cvReleaseImage(&src_float);
  return percent;

}
float Color_Detection_Pro(IplImage* src,double &xpositon,double& yposition)
{
  int sum=0,xsum=0,ysum=0;
  float percent=0;
  cvSmooth(src,src,CV_MEDIAN,5,5);
  IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
  cvConvertScale(src, src_float, 1.0, 0.0);
  IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
  cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
  int step = hsv_img->widthStep/sizeof(float);
  int channels = hsv_img->nChannels;
  float * datafloat = (float *)hsv_img->imageData;
  for(int i = 0; i < hsv_img->height; i++)
  {
    for(int j = 0; j < hsv_img->width; j++)
    {
      if(datafloat[i*step + j*channels + 2]>64&&(datafloat[i*step + j*channels + 1]>0.2)&&(datafloat[i*step + j*channels]>30&&datafloat[i*step + j*channels]<90))
      {
        src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
        src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
        src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
        sum++;
        xsum+=(j+1);
        ysum+=(i+1);
      }
      else
      {
        src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
        src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
        src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
      }
    }
  }
  percent=1.0*sum/(hsv_img->height*hsv_img->width);
  if(sum>0)
  {
    xpositon=1.0*xsum/sum;
    yposition=1.0*ysum/sum;
  }
  else
  {
    xpositon=0;
    yposition=0;
  }
  cvReleaseImage(&hsv_img);
  cvReleaseImage(&src_float);
  return percent;
}

IplImage* mycopy(IplImage* src,CvRect r)
{
  CvSize sz;
  sz.width=r.width;
  sz.height= r.height;
  IplImage* dst;
  int d=src->depth;
  int channels=src->nChannels;
  dst=cvCreateImage(sz,d,channels);
  int i=0,j=0;
  for(i=0;i<r.height;i++)
    for(j=0;j<r.width*channels;j++)
    {
      dst->imageData[i*(dst->widthStep)+j]=src->imageData[(i+r.y)*(src->widthStep)+j+r.x*(channels)];
    }
  return dst;
}
//函数：norm
//功能：归一化为32*32的图像，并返回
//无可调参数
IplImage *norm(IplImage* img)
{
  IplImage *dst = 0; //目标图像指针
  CvSize dst_cvsize; //目标图像尺寸
  dst_cvsize.width = 32; //目标图像的宽为固定 32 像素
  dst_cvsize.height = 32;//目标图像的高为固定 32 像素
  dst = cvCreateImage( dst_cvsize, img->depth, img->nChannels); //构造目标图象
  cvResize(img, dst, CV_INTER_LINEAR);

  return dst;
}
IplImage *norm(IplImage* img,int n,int m)
{ 
  IplImage *dst = 0; //目标图像指针  
  CvSize dst_cvsize; //目标图像尺寸
  dst_cvsize.width = m; //目标图像的宽为固定 m 像素
  dst_cvsize.height = n;//目标图像的高为固定 n 像素
  dst = cvCreateImage( dst_cvsize, img->depth, img->nChannels); //构造目标图象
  cvResize(img, dst, CV_INTER_LINEAR);

  return dst;
}
//函数：divide
//功能提取出图像中需要的有效信息，并返回横纵区间（仅用于经过二值化处理的图像）
//参数：threshould1：连续有效行数大于threshould1为边界
//		threshould2:某行数求和大于threshould2为有效行
//      size[0-1]分别为纵上下限，size[2-3]分别为横上下限
IplImage* divide(IplImage* img,int threshould1,int threshould2)
{
  if(img->nChannels!=1)return NULL;
  int *size;
  size=new int[4];
  std::fill_n(size,4,0);
  int *sum_width,*sum_high;
  sum_width=new int[img->height];
  sum_high=new int[img->width];
  std::fill_n(sum_width,img->height,0);
  std::fill_n(sum_high,img->width,0);
  int i=0,j=0,s=0;
  for(i=0;i<img->height;++i)
  {
    for(j=0;j<img->width;++j)
    {
      if(!((img->imageData)[i*(img->widthStep)+j]))sum_width[i]+=1;
    }

  }
  for(i=0;i<img->height;++i)
  {
    if(sum_width[i]>threshould2)s++;
    else s=0;
    if(s>=threshould1)
    {
      size[0]=i-threshould1+1;
      break;
    }
  }
  s=0;
  for(i=img->height-1;i>=0;--i)
  {
    if(sum_width[i]>threshould2)s++;
    else s=0;
    if(s>=threshould1)
    {
      size[1]=i+threshould1-1;
      break;
    }
  }

  for(i=0;i<img->width;++i)
  {
    for(j=size[0];j<size[1];++j)
    {
      if(!((img->imageData)[j*(img->width)+i]))sum_high[i]+=1;
    }
  }
  s=0;
  for(i=0;i<img->width;++i)
  {
    if(sum_high[i]>threshould2)s++;
    else s=0;
    if(s>=threshould1)
    {
      size[2]=i-threshould1+1;
      break;
    }
  }
  s=0;
  for(i=img->width-1;i>=0;--i)
  {
    if(sum_high[i]>threshould2)s++;
    else s=0;
    if(s>=threshould1)
    {
      size[3]=i+threshould1-1;
      break;
    }
  }

  CvSize dstsize;
  dstsize.height=size[1]-size[0]+1;
  dstsize.width=size[3]-size[2]+1;
  IplImage* dst=cvCreateImage(dstsize, IPL_DEPTH_8U, 1);
  for(i=0;i<dstsize.height;i++)
    for(j=0;j<dstsize.width;j++)
    {
      dst->imageData[i*(dst->widthStep)+j]=img->imageData[(size[0]+i)*(img->widthStep)+size[2]+j];
    }
  return dst;
}
void  divide(IplImage* img,CvRect r,CvRect &rec)
{
  int i=0,j=0;
  int flag=0;
  rec.x=r.x,rec.y=r.y,rec.height=r.height,rec.width=r.width;
  for(i=r.y;i<=r.y+r.height;++i)
  {
    for(j=r.x;j<=r.x+r.width;++j)
    {
      if(((img->imageData)[i*(img->widthStep)+j]&0x00ff)<10)
      {
        rec.y=i;
        flag=1;
        break;
      }
    }
    if(flag==1)break;
  }
  flag=0;
  for(i=r.y+r.height;i>=r.y;--i)
  {
    for(j=r.x;j<=r.x+r.width;++j)
    {
      if(((img->imageData)[i*(img->widthStep)+j]&0x00ff)<10)
      {
        rec.height=i-rec.y;
        flag=1;
        break;
      }
    }
    if(flag==1)break;
  }
  flag=0;
  for(j=r.x;j<=r.x+r.width;++j)
  {
    for(i=r.y;i<=r.y+r.height;++i)
    {
      if(((img->imageData)[i*(img->widthStep)+j]&0x00ff)<10)
      {
        rec.x=j;
        flag=1;
        break;
      }
    }
    if(flag==1)break;
  }
  flag=0;
  for(j=r.x+r.width;j>=r.x;--j)
  {
    for(i=r.y;i<=r.y+r.height;++i)
    {
      if(((img->imageData)[i*(img->widthStep)+j]&0x00ff)<10)
      {
        rec.width=j-rec.x;
        flag=1;
        break;
      }
    }
    if(flag==1)break;
  }
}
//函数：domask
//功能：基于提取到的轮廓进行背景滤除，函数本身就对图像进行了改变
//说明：将所有的图形背景滤除写到了一个函数里面，通过flag进行区别
//可调参数：case 2、3、4中倒数第二行加入了函数cvErode，为腐蚀图像
//			其中最后一个数字可改，越大表示腐蚀次数越多，剩下的也就越小。主要用来缩小三角形提取的图像，以去除外边框影响。
//			如果觉得圆提取也不够理想，可自行加入
void domask(int flag,IplImage* pImg,CvSeq* contours)
{
  IplImage *pImg_copy = cvCloneImage(pImg);
  IplImage *pImg_mask = cvCreateImage(cvGetSize(pImg),IPL_DEPTH_8U,1);
  CvSeqReader reader;
  cvZero(pImg_mask);
  int i;
  switch(flag)
  {
    case 1:
      {	
        for( i = 0; i < contours->total; i++ )
        {
          float *p = (float*)cvGetSeqElem(contours,i);
          cvCircle(pImg_mask,cvPoint(cvRound(p[0]),cvRound(p[1])),cvRound(p[2]),CV_RGB(255,255,255),-1,8,0);
          cvThreshold( pImg_mask, pImg_mask, 230, 255, CV_THRESH_BINARY );
          cvErode(pImg_mask, pImg_mask, NULL, 2);
        }
        cvZero(pImg);
        cvCopy(pImg_copy,pImg,pImg_mask);
        break;
      }
    case 2:
      {
        cvStartReadSeq( contours, &reader, 0 );
        for( i = 0; i < contours->total; i += 4 )
        {
          CvPoint pt[4], *rect = pt;
          int count = 4;
          CV_READ_SEQ_ELEM( pt[0], reader );
          CV_READ_SEQ_ELEM( pt[1], reader );
          CV_READ_SEQ_ELEM( pt[2], reader );
          CV_READ_SEQ_ELEM( pt[3], reader );
          cvFillPoly( pImg_mask, &rect, &count, 1, CV_RGB(255,255,255), 8 , 0 );
        }
        cvErode(pImg_mask, pImg_mask, NULL,2);//腐蚀函数，数字“2”可视情况改变
        break;
      }
    case 3:
      {
        cvStartReadSeq( contours, &reader, 0 );
        for( i = 0; i < contours->total; i += 8 )
        {
          CvPoint pt[8], *rect = pt;
          int count = 8;
          CV_READ_SEQ_ELEM( pt[0], reader );
          CV_READ_SEQ_ELEM( pt[1], reader );
          CV_READ_SEQ_ELEM( pt[2], reader );
          CV_READ_SEQ_ELEM( pt[3], reader );
          CV_READ_SEQ_ELEM( pt[4], reader );
          CV_READ_SEQ_ELEM( pt[5], reader );
          CV_READ_SEQ_ELEM( pt[6], reader );
          CV_READ_SEQ_ELEM( pt[7], reader );
          cvFillPoly( pImg_mask, &rect, &count, 1, CV_RGB(255,255,255), 8 , 0 );
        }
        cvErode(pImg_mask, pImg_mask, NULL,4);//腐蚀函数
        break;
      }
    case 4:
      {
        cvStartReadSeq( contours, &reader, 0 );
        for( i = 0; i < contours->total; i += 3 )
        {
          CvPoint pt[3], *rect = pt;
          int count = 3;
          CV_READ_SEQ_ELEM( pt[0], reader );
          CV_READ_SEQ_ELEM( pt[1], reader );
          CV_READ_SEQ_ELEM( pt[2], reader );
          cvFillPoly( pImg_mask, &rect, &count, 1, CV_RGB(255,255,255), 8 , 0 );
        }
        cvErode(pImg_mask, pImg_mask, NULL,8);//腐蚀函数
        break;
      }
  }
  cvZero(pImg);
  cvCopy(pImg_copy,pImg,pImg_mask);
  cvReleaseImage(&pImg_mask);
  cvReleaseImage(&pImg_copy);
}
//函数：mdomask
//功能：基于提取到的轮廓进行背景滤除，函数本身就对图像进行了改变，使用pImg1对pImg进行滤除背景
//无可调参数
void mdomask(IplImage* pImg,IplImage* pImg1)
{
  int phigh=pImg->height;
  if(pImg->height!=(pImg1->height))return;
  int pwidth=pImg->width;
  if(pwidth!=(pImg1->width))return;
  int i,j;
  for(i=0;i<phigh;++i)
    for(j=0;j<pwidth;++j)
    {
      if(((pImg1->imageData)[i*(pImg1->widthStep)+j])==(char)(255))
      {
        (pImg->imageData)[i*(pImg->widthStep)+3*j]=0;
        (pImg->imageData)[i*(pImg->widthStep)+3*j+1]=0;
        (pImg->imageData)[i*(pImg->widthStep)+3*j+2]=0;
      }
    }
}
//函数：characteristic
//功能：计算图像的特征值
Mat characteristic(IplImage* img,int flag,int size)
{
  int m=img->height,n=img->width;
  float *cha;
  int k=0;
  int size_opt=33;
  int size2=size*size;
  size2=128;
  int qsize2=size2/4;
  size2=8*16;
  if(flag==1)
  {
    cha=new float[qsize2];
    std::fill_n(cha,qsize2,0);
    for(int i=0;i<m;i+=2)
      for(int j=0;j<n;j+=2)
      {
        if(img->imageData[i*n+j]==1)++cha[k];
        if(img->imageData[i*n+j+1]==1)++cha[k];
        if(img->imageData[(i+1)*n+j]==1)++cha[k];
        if(img->imageData[(i+1)*n+j+1]==1)++cha[k];
        cha[k]/=4;
        cha[k]=2*cha[k]-1;
        k++;
      }
    Mat Data(1,qsize2,CV_32FC1,cha);
    return Data;
  }
  if(flag==2)
  {
    cha=new float[size2];
    std::fill_n(cha,size2,0);
    for(int i=0;i<m;i++)
      for(int j=0;j<n;j++)
      {
        if(img->imageData[i*n+j]==1)cha[k]=1;
        else cha[k]=-1;
        k++;
      }
    Mat Data(1,size2,CV_32FC1,cha);
    return Data;
  }	
  if(flag==3)
  {
    int sum=0;
    int u=0;
    cha=new float[size_opt];
    std::fill_n(cha,size_opt,0);
    for(int i=0;i<m;i+=4)
    {
      for(int j=0;j<n;j+=4)
      {
        for(int l=i;l<i+4;l++)
          for(int h=j;h<j+4;++h)
            cha[k]+=img->imageData[l*n+h];
        cha[k]/=16;
        ++k;
      }
      cha[8+i]/=n;
    }
    for(int i=0;i<m;i++)
    {
      for(int j=0;j<n;j++)
      {
        u=img->imageData[i*n+j];
        sum+=u;
        cha[8+i]+=u;
      }
      cha[8+i]/=n;
    }
    for(int i=0;i<n;i++)
    {
      for(int j=0;j<m;j++)
      {
        u=img->imageData[i*n+j];
        cha[24+i]+=u;
      }
      cha[24+i]/=m;
    }
    cha[32]=1.0*sum/size2;
    Mat Data(1,size_opt,CV_32FC1,cha);
    return Data;
  }
}

int predictNumber(IplImage* img,int size)
{
  int size2=size*size;//size of the matrix
  size2=128;
  int size_opt=33;
#if premode==1
  Mat cha(1,size2/4,CV_32FC1);
#else
#if premode==2
  Mat cha(1,size2,CV_32FC1);
#else
  Mat cha(1,size_opt,CV_32FC1);
#endif
#endif
  Mat res(1,10,CV_32FC1);
  IplImage *dst=0,*dsttmp=0;
  IplImage *imgtmp = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
  CvRect omega = cvRect(15, 10, 95, 105);
  CvRect omega2;
  if(img->nChannels>1)
  {
    cvCvtColor(img,imgtmp,CV_BGR2GRAY); //转换
    dsttmp= norm(imgtmp,128,128);
    //cvSetImageROI(dsttmp, omega);
    //cvShowImage("cut",dsttmp);
    //cvResetImageROI(dsttmp);
    divide(dsttmp,omega,omega2);
    if(omega2.height==0||omega2.width==0)return -1;
    cvSetImageROI(dsttmp, omega2);
    dst=norm(dsttmp,16,8);
  }
  else
  {
    dsttmp= norm(img,128,128);
    divide(dsttmp,omega,omega2);
    cvSetImageROI(dsttmp, omega2);
    dst=norm(dsttmp,16,8);
  }
  cvThreshold( dst, dst, 20, 1, CV_THRESH_BINARY_INV);
  cha=characteristic(dst,premode,8);
  //cout<<cha<<endl;
  cvThreshold( dst, dst, 0, 255, CV_THRESH_BINARY_INV);
  cvShowImage("aaa",dst);
  bp.predict(cha,res);
  int pre=0;
  float mindis=10; 
  for(int i=0;i<10;i++)
  {
    float dis=fabs(1-res.at<float>(0,i));
    //cout<<"dis:"<<i<<":"<<dis<<endl;
    if(mindis>dis)
    {
      pre=i;
      mindis=dis;

    }
  }
  if(mindis>0.5)pre=-1;//没有一个匹配的数字
  cvReleaseImage(&imgtmp);
  cvReleaseImage(&dsttmp);
  cvReleaseImage(&dst);
  return pre;
}
