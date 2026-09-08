/*
 * Created on Fri Aug 13 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once
#include <ros/ros.h>
#include <string>
#include <utility>

static std::string _CutParenthesesNTail(std::string &&pretty_func) {
  size_t pos = pretty_func.find('(');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin() + pos, pretty_func.end());
  }

  pos = pretty_func.find(' ');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin(), pretty_func.begin() + pos + 1);
  }

  pretty_func = "[" + pretty_func + "] ";
  return pretty_func;
}

#define __STR_FUNCTION__ _CutParenthesesNTail(std::string(__PRETTY_FUNCTION__))
#define ROS_INFO_STREAM_FUNC(args) ROS_INFO_STREAM(__STR_FUNCTION__ << args)
#define ROS_WARN_STREAM_FUNC(args) ROS_WARN_STREAM(__STR_FUNCTION__ << args)
#define ROS_INFO_STREAM_FUNC(args) ROS_INFO_STREAM(__STR_FUNCTION__ << args)
#define ROS_ERROR_STREAM_FUNC(args) ROS_ERROR_STREAM(__STR_FUNCTION__ << args)

template <typename T> T getParam(const ros::NodeHandle &nh, std::string name, T default_val) {
  T res = nh.param(name, default_val);
//   ROS_INFO_STREAM(nh.getNamespace() << "/" << name << ": " << res);
  return res;
}

template <typename rosTimeType>
std::string stampToDateString(rosTimeType stamp, std::string format="%Y-%m-%d_%H-%M-%S", size_t fractional_digits=4)
{
	const int output_size = 100;
	char output[output_size];
	std::time_t raw_time = static_cast<time_t>(stamp.sec);
	struct tm* timeinfo = localtime(&raw_time);
	std::strftime(output, output_size, format.c_str(), timeinfo);
	std::stringstream ss; 
	ss << std::setw(9) << std::setfill('0') << stamp.nsec;  

	std::string ret = std::string(output) + "_" + ss.str().substr(0, fractional_digits);
	return ret;
}