//程序思想：基于图形外观区并通过神经网络算法进行图像识别
//			三角形则是直接通过颜色判断

//调整说明：为了达到比较好的效果，应该在小车的测试环境里面对程序一些参数进行修改，可调整的参数已在各个函数前面注释说明

//符号说明：使用flag区别各种图形，其中各个数字分别代表：0:LEFT 1:RIGHT 2:STOP 3:P 4：无


#include<time.h>
#include "NumberTrain.h"
#include "string.h"
#include <fstream>
//主函数
int main()
{
	
	//初始化网络并不需要每次调用
	cnn(8);


	CvANN_MLP bp;  
	//bpModel.xml为没经过边缘处理的训练结果
	//bpModel1.xml为经过canny边缘h处理后在训练的结果
	//bp.load("bpModel.xml");
	//CvCapture* pCapture = NULL;
	//pCapture = cvCaptureFromCAM(0);

		
		//mdomask(pImg,pImg_copy);

		//背景复杂不单一时才采用腐蚀和膨胀的方法去除毛刺噪声
		/*cvDilate(pImg_copy,pImg_copy,element,1 );
		cvErode( pImg_copy,pImg_copy,element,1);*/

	return 0;
}

