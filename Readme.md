Building Environment

1.Ubunut 14.04 (or 12.04)
http://my.oschina.net/zhaoqian/blog/406536
no obstacles

2.ros--Indigo (or Hydro)
http://www.jianshu.com/p/04be841e2293
no obstacles

3.autonomy
http://ardrone-autonomy.readthedocs.io/en/latest/installation.html
use method 1
no obstacles

4.VisualSLAM
https://github.com/danping/LibVisualSLAM
it needs libatlas-dev, so install sudo libatlas-dev first:
sudo apt-get install libatlas-dev

if you meet "opencv2/nonfree/features2d.hpp: no such file exist"
try this:
sudo apt-get update
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update
sudo apt-get install libopencv-nonfree-dev

5.Keyboard
sudo apt-get install ros-indigo-keyboard

6.Download program from my github
https://github.com/MozhiJiawei/Ardrone_L-H for program1
https://github.com/MozhiJiawei/Ardrone_2  for program2
"cmake ."
"make"

if no glut.h:
sudo apt-get install freeglut3-dev

if no -llapack:
sudo apt-get install liblapack-dev

7.Leaning ROS and Ardrone_autonomy is strongly recomended, before you start actually coding or reading my solution.

