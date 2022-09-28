/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-12 16:43:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 17:13:10
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

/**
 * @description:
 * @param {cv::Scalar} scalar (H -> range 0~360, S -> range 0~1, V -> range 0~1)
 * @return {cv::Scalar} (R -> range 0~255,G -> range 0~255,B -> range 0~255)
 */
cv::Scalar HSV2RGB(cv::Scalar scalar)
{
    double R, G, B;
    int Hi = (int)abs(scalar.val[0] / 60.0);
    double f = scalar.val[0] / 60.0 - Hi;
    double a = scalar.val[2] * (1 - scalar.val[1]);
    double b = scalar.val[2] * (1 - f * scalar.val[1]);
    double c = scalar.val[2] * (1 - (1 - f) * scalar.val[1]);

    switch (Hi)
    {
    case 0:
        R = scalar.val[2];
        G = c;
        B = a;
        break;
    case 1:
        R = b;
        G = scalar.val[2];
        B = a;
        break;
    case 2:
        R = a;
        G = scalar.val[2];
        B = c;
        break;
    case 3:
        R = a;
        G = b;
        B = scalar.val[2];
        break;
    case 4:
        R = c;
        G = a;
        B = scalar.val[2];
        break;
    case 5:
        R = scalar.val[2];
        G = a;
        B = b;
        break;
    default:
        break;
    }

    return cv::Scalar(R * 255, G * 255, B * 255);
}