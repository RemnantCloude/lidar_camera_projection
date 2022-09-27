/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-26 22:24:25
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-27 11:01:12
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/pointcloud_process.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _POINTCLOUD_PROCESS_H_
#define _POINTCLOUD_PROCESS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>

void pointcloudPassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
std::vector<pcl::PointIndices> pointcloudEuclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, double cluster_tolerance, int min_cluster_size, int max_cluster_size);
void pointcloudWeightCenterPositionCalculation(std::vector<pcl::PointXYZI> points, pcl::PointXYZ &center);
void pointcloudWeightCenterPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointXYZ &center);
void pointcloudAABBPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointXYZI &min_point_AABB, pcl::PointXYZI &max_point_AABB);
// void pointcloudWeightOBBPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);

#endif