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
//#include <numeric>
#include <algorithm>
//#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

//#include <termios.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

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

        struct InitialParameters
        {
            std::string camera_topic;
            std::string lidar_topic;
            cv::Mat intrinsic;
            cv::Mat projection;
            cv::Mat distortion;
            cv::Mat RT;
        } params;

        struct PointcloudFilter
        {
            double xmin;
            double xmax;
            double ymin;
            double ymax;
            double zmin;
            double zmax;
        } pc_region;

        struct Target
        {
            struct BoundingBox boundingbox;     // yolov5候选框
            std::vector<pcl::PointXYZI> points; // 点云
            std::vector<cv::Point> pts;         // 投影点
            pcl::PointXYZ position;             // 几何中心点
        };

        std::string camera_topic;
        std::string lidar_topic;
        image_transport::Publisher image_publisher;
        ros::Subscriber boundingBoxes_sub;
        // tf::TransformBroadcaster tr_br;

        cv::Mat mask;
        std::vector<Target> targets; // 检测到的目标

        void initParams();
        void imageCallback(const sensor_msgs::Image::ConstPtr &img, cv::Mat &undistort_img);
        void pointcloudCluster(pcl::PointXYZI point, cv::Point pt);
        void calculatePointcloudPosition();
        void pointcloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        cv::Mat drawPicture(cv::Mat &img);
        void projectionPublish(cv::Mat &img);
        void projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc);
        void yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes);

    public:
        Projector();
    };
}

#endif