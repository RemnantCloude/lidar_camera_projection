/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-30 16:05:42
 * @FilePath: /test_ws/src/lidar_camera_projection/include/lidar_camera_projection/project.h
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#ifndef _PROJECT_H_
#define _PROJECT_H_

#include "lidar_camera_projection/settings.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

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

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

namespace Projection
{
    class Projector
    {
    private:
        struct Flag
        {
            bool SHOW_TIME;
            bool SHOW_IMAGE;
            bool USE_NUSCENES;
        } flag;

        struct Camera
        {
            std::string TOPIC;
            std::string FRAME_ID;
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
            int RINGS;
            double max_distance;

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
            cv::Mat lidar2cameraM;
            cv::Mat lidar2imageM;
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

            PointCloud::Ptr pc_Ptr;
            std::vector<PointType> points;
            std::vector<float> depths; // 距离
            pcl::PointXYZ center;      // 几何中心点
            PointType min_point_AABB;
            PointType max_point_AABB;

            YOLOV5Target()
            {
                this->pc_Ptr.reset(new PointCloud());
            };
        };

        struct ECTarget
        {
            PointCloud::Ptr pc_Ptr;
            pcl::PointXYZ center; // 几何中心点
            PointType min_point_AABB;
            PointType max_point_AABB;

            ECTarget()
            {
                this->pc_Ptr.reset(new PointCloud());
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

        tf::TransformListener tf_listener;

        cv::Mat undistort_img;
        PointCloud::Ptr cloud_from_lidar;
        PointCloud::Ptr cloud_in_image;
        PointCloud::Ptr cloud_in_image_no_ground;
        std::vector<std::vector<PointType>> points_in_ring;
        std::vector<YOLOV5Target> yolov5_targets; // YOLOV5检测到的目标
        std::vector<ECTarget> ec_targets;
        // std::vector<Point> virtual_pointcloud;

        void initParamsFromYAML();
        void initClassMember();

        void pointcloudImageFilter(PointCloud::Ptr cloud);
        void pointcloudYOLOV5BoundingBoxFilter(PointCloud::Ptr cloud);
        void pointcloudGroundFilter(PointCloud::Ptr cloud);

        void pointcloudEuclideanClusterForYOLOV5();
        void pointcloudEuclideanClusterFor3D(PointCloud::Ptr cloud);
        void pointcloudRingCluster(PointCloud::Ptr cloud);

        // void virtualPointCloudGenerate(cv::Mat image, std::vector<Point> real_pc, std::vector<Point> vitual_pc);

        void drawPoint(cv::Mat &img, PointType point);
        void drawPictureFromPointCloud(cv::Mat &img);
        void drawPictureFromYOLOV5(cv::Mat &img);
        void drawPictureFrom3D(cv::Mat &img);

        void boundingBoxArrayPublish();
        void cloudInImagePublish(PointCloud::Ptr cloud);

        void imageCallback(const sensor_msgs::Image::ConstPtr &img);
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc);
        void projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc);
        void yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes);

    public:
        Projector();
    };
}

#endif