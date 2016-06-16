#include "AffineTransform.h"
const char* wndname = "Square Detection";
int Max_X=640,Max_y=360;//depends on camera
int thresh_Af= 20, N = 11;
vector<double> GetYawFromFrontCam(vector<vector<Point> > &squares,float scale)
{
  vector<double> Angle;
  Angle.clear();
  int n=squares.size();
  double angletmp=0,circumference=0,dis=2;
  for( size_t i = 0; i < n; i++ )
  {
    double x=0,y=0;
    for(int j=0; j<(int)squares[i].size();j++)
    {
      Point* p = &squares[i][j];
      x+=(double)(p->x);
      y+=(double)(p->y);
    }   
    x/=n;
    y/=n;
    double deltaX=abs(x-Max_X/2);
    Angle.push_back(atan(deltaX*scale/dis));     	
  }
  return Angle;
}


// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void findSquares( const Mat& image, vector<vector<Point> >& squares,int mode)
{
  squares.clear();

  Mat pyr, timg, gray0(image.size(), CV_8U), gray;

  // down-scale and upscale the image to filter out the noise
  pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
  pyrUp(pyr, timg, image.size());
  vector<vector<Point> > contours;

  // find squares in every color plane of the image
  for( int c = 0; c < 3; c++ )
  {
    int ch[] = {c, 0};
    mixChannels(&timg, 1, &gray0, 1, ch, 1);

    // try several threshold levels
    for( int l = 0; l < N; l++ )
    {
      // hack: use Canny instead of zero threshold level.
      // Canny helps to catch squares with gradient shading
      if( l == 0 )
      {
        // apply Canny. Take the upper threshold from slider
        // and set the lower to 0 (which forces edges merging)
        Canny(gray0, gray, 0, thresh_Af, 3);
        // dilate canny output to remove potential
        // holes between edge segments
        dilate(gray, gray, Mat(), Point(-1,-1));
      }
      else
      {
        // apply threshold if l!=0:
        //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
        gray = gray0 >= (l+1)*255/N;
      }

      // find contours and store them all as a list
      findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

      vector<Point> approx;

      // test each contour
      for( size_t i = 0; i < contours.size(); i++ )
      {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&isContourConvex(Mat(approx)) )
        {
          if(mode==3)
          {
            if(fabs(contourArea(Mat(approx))) > 800)
            {
              double maxCosine = 0;

              for( int j = 2; j < 5; j++ )
              {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
              }

              // if cosines of all angles are small
              // (all angles are ~90 degree) then write quandrange
              // vertices to resultant sequence
              if( maxCosine < 0.6 )
              {
                double scale=1.0*((approx[0].x-approx[1].x)*(approx[0].x-approx[1].x)+(approx[0].y-approx[1].y)*(approx[0].y-approx[1].y))/
                  (1.0*(approx[1].x-approx[2].x)*(approx[1].x-approx[2].x)+(approx[1].y-approx[2].y)*(approx[1].y-approx[2].y));
                if(scale<2&&scale>0.5)
                  squares.push_back(approx);
              }
            }

          }
          else
          {
            if(fabs(contourArea(Mat(approx))) > 1000)
            {
              double maxCosine = 0;

              for( int j = 2; j < 5; j++ )
              {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
              }

              // if cosines of all angles are small
              // (all angles are ~90 degree) then write quandrange
              // vertices to resultant sequence
              if( maxCosine < 0.3 )
              {

                double scale=1.0*((approx[0].x-approx[1].x)*(approx[0].x-approx[1].x)+(approx[0].y-approx[1].y)*(approx[0].y-approx[1].y))/
                  (1.0*(approx[1].x-approx[2].x)*(approx[1].x-approx[2].x)+(approx[1].y-approx[2].y)*(approx[1].y-approx[2].y));
                if(scale<2&&scale>0.5)
                  if((approx[0].x-approx[1].x)*(approx[0].x-approx[1].x)+(approx[0].y-approx[1].y)*(approx[0].y-approx[1].y)>10000)
                    squares.push_back(approx);
              }

            }
          }
        }
      }
    }
  }
}


// the function draws all the squares in the image
void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
  for( size_t i = 0; i < squares.size(); i++ )
  {
    const Point* p = &squares[i][0];
    int n = (int)squares[i].size();
    polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
  }

  imshow(wndname, image);
}

CvMat *warpPerspective(IplImage *src, IplImage *dst,vector<Point> Top)
{
  //dst大小限制为64*64
  CvPoint2D32f srcTri[4], dstTri[4];
  CvMat* warp_mat = cvCreateMat( 3, 3, CV_32FC1 );
  int flag[4]={0};
  for(int i=0;i<4;i++)
  {
    srcTri[i].x=Top[i].x;
    srcTri[i].y=Top[i].y;
  }
  for(int i=0;i<4;i++)
  {
    for(int j=i+1;j<4;j++)
    {
      if(srcTri[i].x>srcTri[j].x) flag[i]+=10;
      else flag[j]+=10;
      if(srcTri[i].y>srcTri[j].y) flag[i]+=1;
      else flag[j]+=1;
    }
  }
  //sort the points with the src points
  for(int i=0;i<4;i++)
  {
    if(flag[i]>=20)dstTri[i].x=127;
    else dstTri[i].x=0;
    if((flag[i]%10)>=2)dstTri[i].y=127;
    else dstTri[i].y=0;
  }

  cvGetPerspectiveTransform( srcTri, dstTri, warp_mat );  //calculate the transform from the points  
  cvWarpPerspective( src, dst, warp_mat );  //do the perspective
  return  warp_mat; 
}


