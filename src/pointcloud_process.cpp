/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-26 22:21:52
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-10-04 16:53:59
 * @FilePath: /test_ws/src/lidar_camera_projection/src/pointcloud_process.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/pointcloud_process.h"

void pointcloudPassThroughFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
    pcl::PassThrough<PointType> passthrough;
    passthrough.setKeepOrganized(false);

    passthrough.setInputCloud(cloud_in);
    passthrough.setFilterFieldName("x");
    passthrough.setFilterLimits(xmin, xmax);
    passthrough.filter(*cloud_out);

    passthrough.setInputCloud(cloud_out);
    passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(ymin, ymax);
    passthrough.filter(*cloud_out);

    passthrough.setInputCloud(cloud_out);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(zmin, zmax);
    passthrough.filter(*cloud_out);
}

std::vector<pcl::PointIndices> pointcloudEuclideanCluster(PointCloud::Ptr cloud_in, double cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
    std::vector<pcl::PointIndices> cluster_indices;
    if (cloud_in->points.empty() == true)
        return cluster_indices;

    pcl::search::KdTree<PointType>::Ptr kd_tree(new pcl::search::KdTree<PointType>);
    kd_tree->setInputCloud(cloud_in);

    pcl::EuclideanClusterExtraction<PointType> extractor;

    extractor.setClusterTolerance(cluster_tolerance);
    extractor.setMinClusterSize(min_cluster_size);
    extractor.setMaxClusterSize(max_cluster_size);
    extractor.setSearchMethod(kd_tree);
    extractor.setInputCloud(cloud_in);
    extractor.extract(cluster_indices);

    return cluster_indices;
}

void pointcloudWeightCenterPositionCalculation(std::vector<PointType> points, pcl::PointXYZ &center)
{
    double center_x = 0, center_y = 0, center_z = 0;
    int count = 0;
    for (auto point : points)
    {
        center_x += point.x;
        center_y += point.y;
        center_z += point.z;
        count++;
    }
    center.x = center_x / count;
    center.y = center_y / count;
    center.z = center_z / count;
}

void pointcloudWeightCenterPositionCalculation(PointCloud::Ptr cloud_in, pcl::PointXYZ &center)
{
    double center_x = 0, center_y = 0, center_z = 0;
    int count = 0;
    for (auto point : cloud_in->points)
    {
        center_x += point.x;
        center_y += point.y;
        center_z += point.z;
        count++;
    }
    center.x = center_x / count;
    center.y = center_y / count;
    center.z = center_z / count;
}

// high cost, 150ms
void pointcloudAABBPositionCalculation(PointCloud::Ptr cloud_in, PointType &min_point_AABB, PointType &max_point_AABB)
{
    pcl::MomentOfInertiaEstimation<PointType> feature_extractor;
    feature_extractor.setInputCloud(cloud_in);
    feature_extractor.compute();

    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
}

// void pointcloudWeightOBBPositionCalculation(PointCloud::Ptr cloud_in)
// {
//     pcl::MomentOfInertiaEstimation<PointType> feature_extractor;
//     feature_extractor.setInputCloud(cloud_in);
//     feature_extractor.compute();

//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;

//     feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
// }