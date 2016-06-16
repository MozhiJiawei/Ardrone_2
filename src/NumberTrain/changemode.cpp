#include <stdio.h>
#include <fstream>
using namespace std;

ifstream fin("data2(128)(numberWhite).txt");
ofstream fout("data1(128)(numberWhite).txt");
int main()
{
	int size2=128;
	float *cha=new float[size2]; 
	float out=0;
	int i=0;
	for(i=0;i<10;i++)
	{
		for(int j=0;j<size2;j++)fin>>cha[j];
		int k=0;
		for(int h=0;h<16;h+=2)
			for(int l=0;l<8;l+=2)
			{
				if(cha[h*8+l]==1)++out;
				if(cha[h*8+l+1]==1)++out;
				if(cha[(h+1)*8+l]==1)++out;
				if(cha[(h+1)*8+l+1]==1)++out;
				out/=4;
				out=2*out-1;
				k++;
				fout<<out<<" ";
				out=0;
			}
		fout<<endl;
	}
}
