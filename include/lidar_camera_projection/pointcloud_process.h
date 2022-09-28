/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-26 22:24:25
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 14:47:45
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/pointcloud_process.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _POINTCLOUD_PROCESS_H_
#define _POINTCLOUD_PROCESS_H_

#include "lidar_camera_projection/settings.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>

void pointcloudPassThroughFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
std::vector<pcl::PointIndices> pointcloudEuclideanCluster(PointCloud::Ptr cloud_in, double cluster_tolerance, int min_cluster_size, int max_cluster_size);
void pointcloudWeightCenterPositionCalculation(std::vector<PointType> points, pcl::PointXYZ &center);
void pointcloudWeightCenterPositionCalculation(PointCloud::Ptr cloud_in, pcl::PointXYZ &center);
void pointcloudAABBPositionCalculation(PointCloud::Ptr cloud_in, PointType &min_point_AABB, PointType &max_point_AABB);
// void pointcloudWeightOBBPositionCalculation(PointCloud::Ptr cloud_in);

#endif