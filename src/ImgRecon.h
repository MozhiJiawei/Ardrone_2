// ImgRecon.h: interface for the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
#define AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <math.h>
#include <highgui.h>
#include <cv.h>
#include <string>
using namespace std;
class ImgRecon  
{
  public:
    ImgRecon(IplImage *img);//give a colorful picture
    virtual ~ImgRecon();
    void ReInit(IplImage *img);//change picture
    int GetNumber();//get the number in the picture,-1 is not recon
    CvPoint GetCenterPoint();//get contour center
    int GetContourArea();//get contour area
    bool ContourExist();//is contour exist?
    float GetTopPointDiff();//>0,left higher;<0,right higher;return arcsin

  private:
    int NumResult;//判断所得数字
    bool ConExist;//轮廓是否存在
    int ConArea;//轮廓面积
    CvPoint Center;//轮廓中心点
    CvPoint2D32f corners1[4];//四个顺时针排列的角点，第一个为左上
    string SamLoca[10];//样本位置
    int NumDetec(const IplImage* dst);//数字判断函数
    IplImage 
      *src;//灰度图，二值化，腐蚀
    //	*dst;//复制src后找轮廓，存放标准化后的图片
    //	*dst1;//画拟合曲线，找角点

};

#endif // !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
