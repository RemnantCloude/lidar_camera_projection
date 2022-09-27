/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-26 22:21:52
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-27 14:40:39
 * @FilePath: /test_ws/src/lidar_camera_projection/src/pointcloud_process.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/pointcloud_process.h"

void pointcloudPassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
    pcl::PassThrough<pcl::PointXYZI> passthrough;
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

std::vector<pcl::PointIndices> pointcloudEuclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, double cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
    std::vector<pcl::PointIndices> cluster_indices;
    if (cloud_in->points.empty() == true)
        return cluster_indices;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    kd_tree->setInputCloud(cloud_in);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> extractor;

    extractor.setClusterTolerance(cluster_tolerance);
    extractor.setMinClusterSize(min_cluster_size);
    extractor.setMaxClusterSize(max_cluster_size);
    extractor.setSearchMethod(kd_tree);
    extractor.setInputCloud(cloud_in);
    extractor.extract(cluster_indices);

    return cluster_indices;
}

void pointcloudWeightCenterPositionCalculation(std::vector<pcl::PointXYZI> points, pcl::PointXYZ &center)
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

void pointcloudWeightCenterPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointXYZ &center)
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

void pointcloudAABBPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointXYZI &min_point_AABB, pcl::PointXYZI &max_point_AABB)
{
    pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud_in);
    feature_extractor.compute();

    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
}

// void pointcloudWeightOBBPositionCalculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
// {
//     pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
//     feature_extractor.setInputCloud(cloud_in);
//     feature_extractor.compute();

//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;

//     feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
// }