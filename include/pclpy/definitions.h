#ifndef _PCLPY_DEFINITIONS_H
#define _PCLPY_DEFINITIONS_H

#include <boost/python.hpp>
#include <pclpy/wrapper.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <string>

typedef pcl::PointXYZRGB PointT;

#endif // _PCLPY_DEFINITIONS_H
