// ImgRecon.cpp: implementation of the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "ImgRecon.h"
#include <iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ImgRecon::ImgRecon(IplImage *img)
{
#ifdef WIN32
  {
  SamLoca[0]="samples\\sample0.bmp";SamLoca[5]="samples\\sample5.bmp";//样本存放地址
  SamLoca[1]="samples\\sample1.bmp";SamLoca[6]="samples\\sample6.bmp";
  SamLoca[2]="samples\\sample2.bmp";SamLoca[7]="samples\\sample7.bmp";
  SamLoca[3]="samples\\sample3.bmp";SamLoca[8]="samples\\sample8.bmp";
  SamLoca[4]="samples\\sample4.bmp";SamLoca[9]="samples\\sample9.bmp";
  };
#else
  {
  SamLoca[0]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample0.bmp";//样本存放地址
  SamLoca[5]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample5.bmp";
  SamLoca[1]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample1.bmp";
  SamLoca[6]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample6.bmp";
  SamLoca[2]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample2.bmp";
  SamLoca[7]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample7.bmp";
  SamLoca[3]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample3.bmp";
  SamLoca[8]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample8.bmp";
  SamLoca[4]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample4.bmp";
  SamLoca[9]="/home/mozhi/catkin_ws/src/Ardrone_L-H/src/samples/sample9.bmp";
  };
#endif;
  src = cvCreateImage( cvSize(640,360), IPL_DEPTH_8U, 1);
  Center.x = 320;
  Center.y = 180;
  if(img != NULL){
    ReInit(img);
  }
}

ImgRecon::~ImgRecon()
{

}

//更改下一张图片
void ImgRecon::ReInit(IplImage *img)
{	
  NumResult = -2;
  Center.x = 320;
  Center.y = 180;
  ConArea = 0;
  ConExist = 0;


  IplImage 
    //*src,//灰度图，二值化，腐蚀
    *dst;//复制src后找轮廓
    //*dst1;//画拟合曲线，找角点
  //src = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  //dst1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);

  //cvCvtColor( img, src, CV_BGR2GRAY );//转灰img->src
  //cvThreshold( src, src, 130, 255, CV_THRESH_BINARY);//二值化src
  int a = 0, b = 0, c = 0;//new way to threshold
    unsigned char* p = (unsigned char*)(img->imageData);
    unsigned char* q = (unsigned char*)(src->imageData);
    int i = 0, j = 0;
    for(i=0;i<640;++i)
		{
			for(j=0;j<360;++j)
			{
        a=(int)p[i*img->nChannels+(j)*img->widthStep];
        b=(int)p[i*img->nChannels+(j)*img->widthStep+1];
        c=(int)p[i*img->nChannels+(j)*img->widthStep+2];
        
        if((c-a)>0 && (b-a)>0)
        {
          if((a+b+c)<50)
            q[i*src->nChannels+(j)*src->widthStep]=0;
          else
            q[i*src->nChannels+(j)*src->widthStep]=255;          
        }
        else
          q[i*src->nChannels+(j)*src->widthStep]=0;
      }
    }
  cvErode( src, src, NULL, 1);//腐蚀src
  cvCopy( src, dst);//备份src->dst

  //找轮廓dst
  CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
  CvSeq *contour = 0, *cont=0, *contemp=0, *maxcontemp=0;
  int contours = 0;
  contours = cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//CV_RETR_LIST
  int face=0;
  //最大轮廓dst
  for (contemp = contour;contemp != 0; contemp = contemp->h_next)  
  {
    face = fabs(cvContourArea(contemp));
    if(face > ConArea){
      ConArea = face;
      maxcontemp = contemp;
    }		
  }
//cvNamedWindow("dst1", 1);
//cvNamedWindow("src", 1);
  if(ConArea > 10000)//如果面积太小说明没有找到纸片，可以不进行后续处理
  {
    ConExist = 1;		
    //多边形拟合dst1
    cont = cvApproxPoly(maxcontemp, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcontemp)*0.065, 0);//倒数第二个参数为拟合后周长误差，最后参数若为0，只处理src_seq指向的轮廓。1则处理整个双向链表中的所有轮廓。
    //cout << "approxreturn:" <<cont->total<<endl;
    if(cont->total == 4){
      for(i=0;i<4;++i){
        corners1[i] = cvPointTo32f(*(CvPoint*)cvGetSeqElem( cont, i ));        
        //cout<<"corner "<<i<<" "<<corners1[i].x<<" "<<corners1[i].y<<endl;
      }
    //角点顺时针排序，对于图片倾斜太多没办法dst1
    CvPoint2D32f cornertemp;
    int xy=corners1[0].x+corners1[0].y, maxxy=xy, minxy=xy, co2=0, co0=0;
    Center.y = corners1[0].y;
    Center.x = corners1[0].x;
cvCircle( img, cvPointFrom32f(corners1[0]), 7, cvScalar(255,255,255),3);  //
    
    for( i = 1; i < 4; i++ )  
    {
cvCircle( img, cvPointFrom32f(corners1[i]), 7, cvScalar(255,255,255),3);  //
      Center.x += corners1[i].x;
      Center.y += corners1[i].y;
      xy=corners1[i].x+corners1[i].y;
      if(xy>maxxy){
        maxxy=xy;
        co2=i;
      }   
      else{
        if(xy<minxy){
          minxy=xy;
          co0=i;
        }
      }		
    }
    Center.x /= i;
    Center.y /= i;
    

    if(co2!=2){
      cornertemp=corners1[co2];
      corners1[co2]=corners1[2];
      corners1[2]=cornertemp;
      if(co0==2){
        co0=co2;
      }
    }
    if(co0!=0){
      cornertemp=corners1[0];
      corners1[0]=corners1[co0];
      corners1[co0]=cornertemp;
    }
    if((corners1[1].x-corners1[0].x)<100){
      cornertemp=corners1[3];
      corners1[3]=corners1[1];
      corners1[1]=cornertemp;
    }
      
    }
    else{
      int maxx=0,maxy=0,minx=640,miny=360,tx=0,ty=0,maxminxy[4]={-1,-1,-1,-1};//maxxloc=-1,maxyloc=-1,minxloc=-1,minyloc=-1;
      for(i=0;i<cont->total;++i){
        tx = (*(CvPoint*)cvGetSeqElem( cont, i )).x;
        ty = (*(CvPoint*)cvGetSeqElem( cont, i )).y;
        if(tx > maxx){
          maxx = tx;
          maxminxy[0] = i;
        }
        
        if(tx <minx){
          minx = tx;
          maxminxy[1] = i;
        }

        if(ty > maxy){
          maxy = ty;
          maxminxy[2] = i;
        }
        
        if(ty < miny){
          miny = ty;
          maxminxy[3] = i;
        }
      }
      Center.y = (maxy+miny) / 2;
      Center.x = (maxx+minx) / 2;
      
      }
    }
    /*
    cvDrawContours (dst1, cont, cvScalar(255,0,0), cvScalar(255,0,0), CV_FILLED, 4, 8);//maxcontemp
    cvErode( dst1, dst1, NULL, 1);
//cout<<"多边形拟合角点数："<<cont->total<<endl;

    //找角点dst1	
    IplImage *eig_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1), *temp_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1); ;
    int maxcorners=8;

    cvGoodFeaturesToTrack(dst1, eig_image, temp_image, corners1, &maxcorners, 0.01, 50);//最后一个参数为两个点距离最小值	

    //释放不需要的内存
    //cvReleaseImage(&eig_image);
    //cvReleaseImage(&temp_image);
*/    
  
  
  cvCircle( img, Center, 7, cvScalar(255,0,255),3);//在轮廓中心画圆
//cvShowImage("src",src);//show src, dst1
//cvShowImage("dst1",dst1);
  //释放不需要的内存
  //cvReleaseImage(&src);
  //cvReleaseImage(&dst1);
  cvReleaseImage(&dst);
  cvReleaseMemStorage (&storage);
  cvReleaseMemStorage (&storage1);
}

//数字判断函数
int ImgRecon::NumDetec(const IplImage* dst)
{
  int xsmin[10]={68,80,70,71,67,71,68,72,65,70},//样本图片数字的位置
      ysmin[10]={48,55,55,55,57,56,55,56,55,55},
      xsdel[10]={64,44,62,60,67,60,64,58,61,62},
      ysdel[10]={91,89,91,90,87,89,91,88,88,91};

  //框出目标数字部分
  int xmin=0,ymin=0,xmax=200,ymax=200,flag=0,i=0,j=0;
  unsigned char* p = (unsigned char*)(dst->imageData);

  for(i=50;i<100;++i)//col,width,x
  {
    for(j=35;j<130;++j)//row,height,y
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        xmin=i;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(j=35;j<100;++j)
  {
    for(i=50;i<150;++i)
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        ymin=j;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(i=140;i>100;--i)
  {
    for(j=150;j>70;--j)
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        xmax=i;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(j=150;j>100;--j)
  {
    for(i=140;i>100;--i)
    {			
      if((int)p[i+j*dst->widthStep]==0)
      {
        ymax=j;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }
  //cout<<"被检测图像数字位置："<<xmin<<"  "<<xmax<<"  "<<ymin<<"  "<<ymax<<endl;

  //判断数字
  int countdiff=0,k=0,mindiff=5000,tempdiff=0,rnum=-2;
  unsigned char* q;
  IplImage *sam;

  for(k=0;k<10;++k)
  {
    sam = cvLoadImage(SamLoca[k].c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if(!sam)
    {
      cout<<"Can not load sample image "<<k<<endl;
      return -1;
    }
    q = (unsigned char*)(sam->imageData);

    for(i=0;i<=xsdel[k];++i)
    {
      for(j=0;j<=ysdel[k];++j)
      {
        if((int)p[xmin+i+(j+ymin)*dst->widthStep]!=(int)q[xsmin[k]+i+(j+ysmin[k])*sam->widthStep])
          tempdiff++;
      }
    }
    //cout<<"与"<<k<<"的样本差别像素数为："<<tempdiff<<endl;
    if(tempdiff<mindiff) 
    {
      mindiff=tempdiff;
      rnum=k;
    }
    tempdiff = 0;
    cvReleaseImage(&sam);
  }
  if(mindiff>1500) return -1;
  return rnum;
}

//获得图片中的数字
int ImgRecon::GetNumber()
{
  if(NumResult == -2){
    IplImage *dst;
    dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1);
    //投影变换src->dst
    CvMat* warp_mat = cvCreateMat (3, 3, CV_32FC1);
    CvPoint2D32f dstTri[4];
    dstTri[0].x = 0;  
    dstTri[0].y = 0;  
    dstTri[1].x = 200;  
    dstTri[1].y = 0;  
    dstTri[2].x = 200;  
    dstTri[2].y = 200;  
    dstTri[3].x = 0;  
    dstTri[3].y = 200;
    cvGetPerspectiveTransform(corners1, dstTri, warp_mat);	
    cvWarpPerspective(src, dst, warp_mat);
    cvReleaseMat(&warp_mat);

    //判断数字
    NumResult = NumDetec(dst);
    cvReleaseImage(&dst);
  }
  return NumResult;
}

//获得轮廓中心
CvPoint ImgRecon::GetCenterPoint()
{
  return Center;
}

//获取轮廓面积
int ImgRecon::GetContourArea()
{
  return ConArea;
}

//轮廓存在与否
bool ImgRecon::ContourExist()
{
  return ConExist;
}

//>0,left higher;<0,right higher
float ImgRecon::GetTopPointDiff()
{
  return asin((corners1[1].y-corners1[0].y)/sqrt((corners1[0].x-corners1[1].x)*(corners1[0].x-corners1[1].x)+(corners1[1].y-corners1[0].y)*(corners1[1].y-corners1[0].y)));
}
