/*
* RefPoseBr.cpp
*
*  Created on: May 19, 2016
*      Author: ljw
*/

#include "RefPoseBr.h"
#include <iostream>

RefPoseBr::RefPoseBr() {
  _running = false;
  _toQuit = false;
  _threadID = 0;

  pthread_mutex_init(&_mutex, 0);
}

void RefPoseBr::Start() {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&_threadID, &attr, ThreadProc, this);
}

void * RefPoseBr::ThreadProc(void * data) {
  RefPoseBr* broadcaster = (RefPoseBr*)data;
  broadcaster->Loop();
}

void RefPoseBr::Loop() {
  ros::NodeHandle node;
  tf::TransformBroadcaster br;
  vector<tf::StampedTransform>::iterator itr;

  _running = true;
  cout << "starting Broadcasting thread" << endl;

  ros::Rate rate(10.0);
  while (node.ok() && !_toQuit) {
    pthread_mutex_lock(&_mutex);
    for (itr = _ref_pose.begin(); itr != _ref_pose.end(); ++itr) {
      (*itr).stamp_ = ros::Time::now();
      _br.sendTransform(*itr);
    }
    pthread_mutex_unlock(&_mutex);
    rate.sleep();
  }
  cout << "Broadcast Thread exits!" << endl;
  _running = false;
}

RefPoseBr::~RefPoseBr() {
  End();
}

void RefPoseBr::End() {
  if (_running) {
    _toQuit = true;
    std::cout << "waiting the RefPose Broadcasting thread to quit..." 
      << std::endl;

    pthread_mutex_lock(&_mutex);
    pthread_mutex_unlock(&_mutex);
    pthread_join(_threadID, 0);
    std::cout << "RefPose BroadCast thread quit!" << std::endl;
  }
}

vector<tf::StampedTransform>::iterator RefPoseBr::FindPose(
  const string& frame_id, const string& child_frame_id) {

  vector<tf::StampedTransform>::iterator itr;
  for (itr = _ref_pose.begin(); itr != _ref_pose.end(); ++itr) {
    if ((*itr).child_frame_id_ == child_frame_id &&
        (*itr).frame_id_ == frame_id) {

      return itr;
    }
  }
  return _ref_pose.end();
}

void RefPoseBr::NewBr(const tf::Transform input, const std::string& frame_id,
  const std::string& child_frame_id) {

  if (FindPose(frame_id, child_frame_id) == _ref_pose.end()) {
    pthread_mutex_lock(&_mutex);
    _ref_pose.push_back(tf::StampedTransform(input, ros::Time::now(), 
        frame_id, child_frame_id));

    pthread_mutex_unlock(&_mutex);
  }
  else {
    cout << "Trans already exists!!" << endl;
  }
  if (!_running && _ref_pose.size() != 0) {
    Start();
  }
}

void RefPoseBr::SetRefPose(const tf::Transform input,
  const std::string& frame_id,
  const std::string& child_frame_id) {

  vector<tf::StampedTransform>::iterator itr;
  itr = FindPose(frame_id, child_frame_id);
  if (itr != _ref_pose.end()) {
    pthread_mutex_lock(&_mutex);
    (*itr).setData(input);
    pthread_mutex_unlock(&_mutex);
  }
  else {
    cout << "Trans do not exists!! Add it first!!" << endl;
  }
}
