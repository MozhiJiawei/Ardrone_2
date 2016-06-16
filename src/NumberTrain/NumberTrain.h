#ifndef   __NUMBERTRAIN_H__ 
#define   __NUMBERTRAIN_H__ 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#define mode 3//1->256,2->1024,3->optw ways


//函数：cnn
//功能：神经网络，构建初始化网络，并保存
void cnn(int size)
{
	int size2=size*size;
		size2=128;//强制大小
	int qsize2=size2/4;
	int size_opt=33;
	//从文件载入数据  
	#if mode==1
		ifstream fid("data1(128)(numberWhite).txt");
	#else
		#if mode==2
			ifstream fid("data2(128)(numberWhite).txt");
		#else
			ifstream fid("data(opt).txt");
		#endif
	#endif
    #if mode==1
		float *data=new float[10*qsize2]; 
		for(int i=0;i<10*qsize2;++i)fid>>data[i];
		Mat trainData(10,qsize2,CV_32FC1,data);
	#else
		#if mode==2
			float *data=new float[10*size2]; 
			for(int i=0;i<10*size2;++i)fid>>data[i];
			Mat trainData(10,size2,CV_32FC1,data);
		#else
			float *data=new float[10*size_opt]; 
			for(int i=0;i<10*size_opt;++i)fid>>data[i];
			Mat trainData(10,size_opt,CV_32FC1,data);
		#endif		
	#endif
	/*for(int i=0;i<10;++i)
	{
		for(int j=0;j<size_opt;++j)
			cout<<trainData.at<float>(i,j)<<" ";
		cout<<endl;
	}*/
	fid.close();    
  
	int mod=10;
	ifstream fid2("label.txt");  
	float* label = new float[mod*mod];  
	for(int i=0;i<mod*mod;++i)fid2>>label[i];
	Mat trainLabel(mod,mod,CV_32FC1,label);  
	fid2.close();  
	//Setup the BPNetwork  
    CvANN_MLP bp;   
    // Set up BPNetwork's parameters  
    CvANN_MLP_TrainParams params;  
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,1000000,0.00001);  //设置结束条件
    params.train_method=CvANN_MLP_TrainParams::BACKPROP;  
    params.bp_dw_scale=0.1;  
    params.bp_moment_scale=0.1;
    //params.train_method=CvANN_MLP_TrainParams::RPROP;
    //params.rp_dw0 = 0.1;   
    //params.rp_dw_plus = 1.2;   
    //params.rp_dw_minus = 0.5;  
    //params.rp_dw_min = FLT_EPSILON;   
    //params.rp_dw_max = 50.;  
    #if mode==1
		int layers[3] = {qsize2,20,10};
		Mat_<int> layerSize(1,3,layers);    
	#else
		#if mode==2
			int layers[4] = {size2,60,20,10};
			Mat_<int> layerSize(1,4,layers);
		#else 
			int layers[3] = {size_opt,20,10};
			Mat_<int> layerSize(1,3,layers);
		#endif  
	#endif
	
  
  
	bp.create(layerSize,CvANN_MLP::SIGMOID_SYM);  
	bp.train(trainData,trainLabel,Mat(),Mat(),params); 
	#if mode==1 
		bp.save("bpModel1.xml"); //save classifier  
	#else		
		#if mode==2
			bp.save("bpModel2.xml"); //save classifier 
		#else 
			bp.save("bpModel_op.xml"); //save classifier 
		#endif 
	#endif

}
//函数：characteristic
//功能：计算图像的特征值
Mat characteristic(IplImage* img,int flag)
{
	int m=img->height,n=img->width;
	float *cha;
	int k=0;
	
	if(flag==1)
	{
		cha=new float[256];
		fill_n(cha,256,0);
		for(int i=0;i<m;i+=2)
			for(int j=0;j<n;j+=2)
			{
				if(img->imageData[i*n+j]==(char)255)++cha[k];
				if(img->imageData[i*n+j+1]==(char)255)++cha[k];
				if(img->imageData[(i+1)*n+j]==(char)255)++cha[k];
				if(img->imageData[(i+1)*n+j+1]==(char)255)++cha[k];
				cha[k]/=4;
				cha[k]=2*cha[k]-1;
				k++;
			}
		Mat Data(1,256,CV_32FC1,cha);
		return Data;
	}
	if(flag==2)
	{
		cha=new float[1024];
		fill_n(cha,1024,0);
		for(int i=0;i<m;i++)
			for(int j=0;j<n;j++)
			{
				if(img->imageData[i*n+j]==(char)255)cha[k]=1;
				else cha[k]=-1;
				k++;
			}
		Mat Data(1,1024,CV_32FC1,cha);
		return Data;
	}	
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
	cvReleaseImage(&img);

	return dst;
}
Mat PreProcess(IplImage* img)
{
	IplImage *imgtmp = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
    cvCvtColor(img,imgtmp,CV_BGR2GRAY); //转换
	imgtmp=norm(imgtmp);
	cvThreshold( imgtmp, imgtmp, 127, 255, CV_THRESH_OTSU);
	return characteristic(imgtmp,mode);
	
}


#endif
