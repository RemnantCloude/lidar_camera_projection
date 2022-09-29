/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-28 14:42:59
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-29 10:14:21
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/settings.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// This structure should stay the same with the structure in nuscences2bag(https://github.com/clynamen/nuscenes2bag)
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, ring, ring))

// typedef pcl::PointXYZI PointType;
typedef PointXYZIR PointType;
typedef pcl::PointCloud<PointType> PointCloud;

#endif