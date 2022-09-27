/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-27 14:47:41
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/project.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _PROJECT_H_
#define _PROJECT_H_

#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace Projection
{
    class Projector
    {
    private:
        struct Flag
        {
            bool IF_SHOW_TIME;
        } flag;

        struct Camera
        {
            std::string TOPIC;
            int IMAGE_WIDTH;
            int IMAGE_HEIGHT;
            std::vector<double> intrinsicV;
            std::vector<double> projectionV;
            std::vector<double> distortionV;
            cv::Mat intrinsicM;
            cv::Mat projectionM;
            cv::Mat distortionM;
        } camera;

        struct Lidar
        {
            std::string TOPIC;
            std::string FRAME_ID;

            struct PointcloudFilterParams
            {
                double xmin;
                double xmax;
                double ymin;
                double ymax;
                double zmin;
                double zmax;
            } pc_region;

            struct EuclideanClusterParams
            {
                double cluster_tolerance;
                int min_cluster_size;
                int max_cluster_size;
            } ec_cluster_params;
        } lidar;

        struct Transform
        {
            std::vector<double> lidar2cameraV;
            cv::Mat lidar2cameraC;
            cv::Mat lidar2imageC;
        } transform;

        struct YOLOV5Target
        {
            struct YOLOV5BoundingBox
            {
                float probability;
                int xmin;
                int ymin;
                int xmax;
                int ymax;
                int num;
                std::string Class;
            } boundingbox; // yolov5识别框

            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_Ptr;
            std::vector<pcl::PointXYZI> points;
            std::vector<cv::Point> pts; // 投影点
            std::vector<float> depths;  // 距离
            pcl::PointXYZ center;       // 几何中心点
            pcl::PointXYZI min_point_AABB;
            pcl::PointXYZI max_point_AABB;

            YOLOV5Target()
            {
                this->pc_Ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
            };
        };

        struct ECTarget
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_Ptr;
            pcl::PointXYZ center; // 几何中心点
            pcl::PointXYZI min_point_AABB;
            pcl::PointXYZI max_point_AABB;

            ECTarget()
            {
                this->pc_Ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
            };
        };

        int mode;

        ros::NodeHandle nh;

        ros::Subscriber image_sub;
        ros::Subscriber pointcloud_sub;
        ros::Subscriber yolov5_boundingBoxes_sub;

        image_transport::Publisher projected_image_pub;
        ros::Publisher cloud_in_image_pub;
        ros::Publisher lidar_boundingBoxesPosition_pub;
        ros::Publisher lidar_boundingBoxesArray_pub;

        cv::Mat undistort_img;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_from_lidar;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_image;
        std::vector<YOLOV5Target> yolov5_targets; // YOLOV5检测到的目标
        std::vector<ECTarget> ec_targets;
        // std::vector<Point> real_pointcloud;
        // std::vector<Point> real_pointcloud_no_ground;
        // std::vector<Point> virtual_pointcloud;

        void initParamsFromYAML();
        void initClassMember();
        void pointcloudImageFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void pointcloudGroundFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        // void virtualPointCloudGenerate(cv::Mat image, std::vector<Point> real_pc, std::vector<Point> vitual_pc);
        void pointcloudYOLOV5BoundingBoxFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void pointcloudEuclideanClusterForYOLOV5();
        void pointcloudEuclideanClusterFor3D();
        cv::Mat drawPictureFromPointCloud(cv::Mat &img);
        cv::Mat drawPictureFromYOLOV5(cv::Mat &img);
        cv::Mat drawPictureFrom3D(cv::Mat &img);

        void boundingBoxArrayPublish();
        void cloudInImagePublish(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        void imageCallback(const sensor_msgs::Image::ConstPtr &img);
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc);
        void projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc);
        void yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes);

    public:
        Projector();
    };
}

#endif