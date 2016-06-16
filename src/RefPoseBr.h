/*
* RefPoseBr.h
*
*  Created on: May 19, 2016
*      Author: ljw
*/

#ifndef REFPOSEBR_H_
#define REFPOSEBR_H_

#include "pthread.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
using namespace std;

class RefPoseBr {
public:
  RefPoseBr();
  virtual ~RefPoseBr();

  void NewBr(const tf::Transform input, const std::string& frame_id, 
    const std::string& child_frame_id);

  void SetRefPose(const tf::Transform input, const std::string& frame_id,
    const std::string& child_frame_id);
 
private:
  vector<tf::StampedTransform> _ref_pose;
  bool _running;
  bool _toQuit;
  pthread_t _threadID;
  pthread_mutex_t _mutex;
  tf::TransformBroadcaster _br;

  vector<tf::StampedTransform>::iterator FindPose(
    const string& frame_id, const string& child_frame_id);

  void Start();
  void Loop();
  void End();
  static void * ThreadProc(void* data);

};
//class IMURecorder {
//public:
//  IMUData curData;
//  bool running;
//  bool toQuit;
//  pthread_t threadId;
//  pthread_mutex_t _mutex;
//  pthread_cond_t _cond;
//
//  pthread_mutex_t _opmutex;
//  deque<IMUData> imuQ;
//
//  const char* _filePath;
//  ofstream log;
//  bool _recording;
//
//  int maxCacheNum;
//  std::deque<IMUData> cachedImuData;
//public:
//  IMURecorder(const char* fileName);
//  virtual ~IMURecorder();
//
//  static void * threadProc(void* data);
//  void addBack(const IMUData& data);
//  bool popFront(IMUData& data);
//  void getCurData(IMUData& data);
//  void getCachedAvg(IMUData& data);
//  void getCachedMeas(std::vector<IMUData>& cachedMeas);
//
//  void start();
//  void loop();
//  void end();
//};

#endif /* REFPOSEBR_H_ */
