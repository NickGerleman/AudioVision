#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/make_shared.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <OpenNI2/OpenNI.h>


using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using IntensityCloud = pcl::PointCloud<pcl::PointXYZI>;
using Timestamp = std::chrono::high_resolution_clock::time_point;
