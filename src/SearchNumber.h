#ifndef _SEARCHNUMBER_H
#define _SEARCHNUMBER_H

#include "stdlib.h"
#include "AffineTransform.h"
#include "PredictNumber.h"

struct numberec
{
  bool isfind;
  int positionX;
  int positionY;
};
class number
{
  public:
    number(int n=3,int m=5):_width(n),_high(m)
  {
    for(int i=0;i<10;i++)
    {
      _number[i].isfind=false;
      _number[i].positionX=0;
      _number[i].positionY=0;
    }
  }
    void add(int num,int n,int m)
    {
      if(num<0)return;
      _number[num].isfind=true;
      _number[num].positionX=n;
      _number[num].positionY=m;
    }
    void getPositon(int num,int &x,int &y)
    {
      if(num<0)return;
      x=_number[num].positionX;
      y=_number[num].positionY;		
    }
    bool isNumFind(int num)
    {
      if(num<0)return false;
      return _number[num].isfind;
    }
  private:
    int _width;
    int _high;
    numberec _number[10];
};
void searchnum(IplImage *img);

#endif
