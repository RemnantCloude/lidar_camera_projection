/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-12 16:43:25
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-10-04 16:04:02
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/algorithm.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include <cmath>

#include <opencv2/opencv.hpp>

template <typename PointT>
float euclideanDistance(PointT point)
{
    float x = point.x, y = point.y, z = point.z;
    return sqrt(x * x + y * y + z * z);
}

template <typename PointT>
cv::Point point2pixel(PointT point, cv::Mat transform)
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

cv::Scalar HSV2RGB(cv::Scalar scalar);
void minRect(std::vector<cv::Point> pts, cv::Point &min, cv::Point &max);

#endif