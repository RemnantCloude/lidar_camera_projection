/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-12 16:43:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 14:46:32
 * @FilePath: /test_ws/src/lidar_camera_projection/src/algorithm.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/algorithm.h"

float euclideanDistance(PointType point)
{
    float x = point.x, y = point.y, z = point.z;
    return sqrt(x * x + y * y + z * z);
}

cv::Point pointcloud2image(PointType point, cv::Mat transform)
{
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    X = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
    Y = transform * X;

    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    return pt;
}