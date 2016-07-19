#Building Environment

###1.[Ubunut 14.04](http://my.oschina.net/zhaoqian/blog/406536 "or 12.04")
no obstacles<br>

###2.[ros--Indigo](http://www.jianshu.com/p/04be841e2293 "or Hydro")
no obstacles<br>

###3.[autonomy](http://ardrone-autonomy.readthedocs.io/en/latest/installation.html)
use method 1<br>
no obstacles<br>

###4.[VisualSLAM](https://github.com/danping/LibVisualSLAM)
it needs libatlas-dev, so install sudo libatlas-dev first:<br>
sudo apt-get install libatlas-dev<br>
<br>
if you meet "opencv2/nonfree/features2d.hpp: no such file exist"<br>
try this:<br>
sudo apt-get update<br>
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree<br>
sudo apt-get update<br>
sudo apt-get install libopencv-nonfree-dev<br>

###5.Keyboard
sudo apt-get install ros-indigo-keyboard<br>

###6.Download program from my github
[Program1](https://github.com/MozhiJiawei/Ardrone_L-H)
[Program2](https://github.com/MozhiJiawei/Ardrone_2)
"cmake ."<br>
"make"<br>
<br>
if no glut.h:<br>
sudo apt-get install freeglut3-dev<br>
<br>
if no -llapack:<br>
sudo apt-get install liblapack-dev<br>
<br>
###7.Learn First
Leaning 'ROS' and 'Ardrone_autonomy' is strongly recomended, before you start actually coding or reading my solution.
