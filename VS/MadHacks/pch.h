#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <OpenNI2/OpenNI.h>


using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using IntensityCloud = pcl::PointCloud<pcl::PointXYZI>;
using Timestamp = std::chrono::high_resolution_clock::time_point;
