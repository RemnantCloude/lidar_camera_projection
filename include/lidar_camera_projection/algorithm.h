/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-12 16:43:25
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 16:47:02
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/algorithm.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "lidar_camera_projection/settings.h"

#include <cmath>

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

float euclideanDistance(PointType point);
cv::Point pointcloud2image(PointType point, cv::Mat transform);
cv::Scalar HSV2RGB(cv::Scalar scalar);

#endif