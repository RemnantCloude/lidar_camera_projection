/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-12 16:43:25
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-24 16:54:05
 * @FilePath: /lidar_camera_projection/include/lidar_camera_projection/algorithm.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include <cmath>

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

float euclideanDistance(pcl::PointXYZI point);
float euclideanDistance(float x, float y, float z);
cv::Point pointcloud2image(pcl::PointXYZI point, cv::Mat transform);

#endif