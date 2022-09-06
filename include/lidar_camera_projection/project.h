/*
 *@author: Cloude Remnant
 *@date: 2022-09-03
 *@description:
 */

#ifndef _PROJECTION_H_
#define _PROJECTION_H_

#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
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
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

namespace Projection
{
    class Projector
    {
    private:
        struct BoundingBox
        {
            float probability;
            int xmin;
            int ymin;
            int xmax;
            int ymax;
            int num;
            std::string Class;
        };

        struct PointcloudFilter
        {
            double xmin;
            double xmax;
            double ymin;
            double ymax;
            double zmin;
            double zmax;
        };

        struct Camera
        {
            std::string TOPIC;
            int IMAGE_WIDTH;
            int IMAGE_HEIGHT;
            std::vector<double> intrinsicV;
            std::vector<double> projectionV;
            std::vector<double> distortionV;
            // Eigen::Matrix3d intrinsicE;
            // Eigen::Matrix3d projectionE;
            // Eigen::Matrix3d distortionE;
            cv::Mat intrinsicC;
            cv::Mat projectionC;
            cv::Mat distortionC;
        } camera;

        struct Lidar
        {
            std::string TOPIC;
            struct PointcloudFilter pc_region;
        } lidar;

        struct Transform
        {
            std::vector<double> lidar2cameraV;
            cv::Mat lidar2cameraC;
        } transform;

        struct Target
        {
            struct BoundingBox boundingbox;     // yolov5候选框
            std::vector<pcl::PointXYZI> points; // 点云
            std::vector<cv::Point> pts;         // 投影点
            pcl::PointXYZ position;             // 几何中心点
        };

        ros::NodeHandle nh;
        ros::Subscriber boundingBoxes_sub;
        image_transport::Publisher image_pub;
        ros::Publisher boundingBoxesPosition_pub;

        std::vector<Target> targets; // 检测到的目标

        void initParams();
        void imageCallback(const sensor_msgs::Image::ConstPtr &img, cv::Mat &undistort_img);
        void pointcloudCluster(pcl::PointXYZI point, cv::Point pt);
        void calculatePointcloudPosition();
        void pointcloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        cv::Mat drawPicture(cv::Mat &img);
        void boundingBoxesPositionPublish();
        void projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc);
        void yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes);

    public:
        Projector();
    };
}

#endif