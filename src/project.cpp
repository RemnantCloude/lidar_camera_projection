/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 17:31:50
 * @FilePath: /test_ws/src/lidar_camera_projection/src/project.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/project.h"
#include "lidar_camera_projection/algorithm.h"
#include "lidar_camera_projection/pointcloud_process.h"

#include <typeinfo>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
// #include <jsk_recognition_msgs/BoundingBox.h>
// #include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace Projection
{
    void Projector::initParamsFromYAML()
    {
        nh.param<int>("mode", mode, 0);
        nh.param<bool>("flag/show_time", flag.SHOW_TIME, true);
        nh.param<bool>("flag/use_nuscenes", flag.USE_NUSCENES, true);

        nh.param<std::string>("camera/topic", camera.TOPIC, "/lbas_image");
        nh.param<std::string>("camera/frame_id", camera.FRAME_ID, "");
        nh.param<int>("camera/image_width", camera.IMAGE_WIDTH, 1440);
        nh.param<int>("camera/image_height", camera.IMAGE_HEIGHT, 1080);
        nh.param<std::vector<double>>("camera/intrinsic", camera.intrinsicV, std::vector<double>());
        nh.param<std::vector<double>>("camera/projection", camera.projectionV, std::vector<double>());
        nh.param<std::vector<double>>("camera/distortion", camera.distortionV, std::vector<double>());
        camera.intrinsicM = cv::Mat(camera.intrinsicV).reshape(0, 3);   // 3*3
        camera.projectionM = cv::Mat(camera.projectionV).reshape(0, 3); // 3*4
        camera.distortionM = cv::Mat(camera.distortionV).reshape(0, 1); // 1*5

        nh.param<std::string>("lidar/topic", lidar.TOPIC, "/rslidar_points");
        nh.param<std::string>("lidar/frame_id", lidar.FRAME_ID, "rslidar");
        nh.param<double>("lidar/filter/xmin", lidar.pc_region.xmin, 0.0);
        nh.param<double>("lidar/filter/xmax", lidar.pc_region.xmax, 10.0);
        nh.param<double>("lidar/filter/ymin", lidar.pc_region.ymin, -10.0);
        nh.param<double>("lidar/filter/ymax", lidar.pc_region.ymax, 10.0);
        nh.param<double>("lidar/filter/zmin", lidar.pc_region.zmin, -10.0);
        nh.param<double>("lidar/filter/zmax", lidar.pc_region.zmax, 10.0);
        nh.param<double>("lidar/ec_extraction/cluster_tolerance", lidar.ec_cluster_params.cluster_tolerance, 0.1);
        nh.param<int>("lidar/ec_extraction/min_cluster_size", lidar.ec_cluster_params.min_cluster_size, 20);
        nh.param<int>("lidar/ec_extraction/max_cluster_size", lidar.ec_cluster_params.max_cluster_size, 25000);
        nh.param<double>("lidar/max_distance", lidar.max_distance, 50.0);

        nh.param<std::vector<double>>("transform/lidar2camera", transform.lidar2cameraV, std::vector<double>());
        transform.lidar2cameraM = cv::Mat(transform.lidar2cameraV).reshape(0, 4); // 4*4

        if (flag.USE_NUSCENES)
            transform.lidar2imageM = camera.projectionM;
        else
            transform.lidar2imageM = camera.projectionM * transform.lidar2cameraM;

        ROS_INFO("Read initial param file successfully.");
    }

    void Projector::initClassMember()
    {
        cloud_from_lidar = boost::make_shared<PointCloud>();
        cloud_in_image = boost::make_shared<PointCloud>();
    }

    void Projector::imageCallback(const sensor_msgs::Image::ConstPtr &img)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            ros::shutdown();
        }

        if (cv_ptr->image.size().width != camera.IMAGE_WIDTH || cv_ptr->image.size().height != camera.IMAGE_HEIGHT)
        {
            ROS_ERROR("Get wrong size of image from <params.yaml>. The received images size is (%d,%d).", cv_ptr->image.size().width, cv_ptr->image.size().height);
            ros::shutdown();
        }
        // 畸变矫正
        if (flag.USE_NUSCENES)
            undistort_img = cv_ptr->image.clone();
        else
            cv::undistort(cv_ptr->image, undistort_img, camera.intrinsicM, camera.distortionM);
    }

    void Projector::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        // if (flag.USE_NUSCENES)
        // {
        //     tf::StampedTransform transform_lidar;
        //     tf::StampedTransform transform_cam;

        //     tf_listener.lookupTransform("/lidar_top", "/base_link",
        //                                 ros::Time(0), transform_lidar);
        //     tf_listener.lookupTransform("/cam_front", "/base_link",
        //                                 ros::Time(0), transform_cam);
        //     Eigen::Matrix4f lidar2base;
        //     Eigen::Matrix4f cam2base;
        //     pcl_ros::transformAsMatrix(transform_lidar, lidar2base);
        //     pcl_ros::transformAsMatrix(transform_cam, cam2base);
        //     Eigen::Matrix4f cam2lidar = cam2base * lidar2base.inverse();

        //     std::cout << "cam2lidar" << std::endl;
        //     std::cout << cam2lidar << std::endl;

        //     // cv::eigen2cv(cam2lidar, transform.lidar2cameraM);
        //     cv::Mat temp(4, 4, cv::DataType<double>::type);
        //     temp = (cv::Mat_<double>(4, 4) << cam2lidar(0, 0), cam2lidar(0, 1), cam2lidar(0, 2), cam2lidar(0, 3), cam2lidar(1, 0), cam2lidar(1, 1), cam2lidar(1, 2), cam2lidar(1, 3), cam2lidar(2, 0), cam2lidar(2, 1), cam2lidar(2, 2), cam2lidar(2, 3), cam2lidar(3, 0), cam2lidar(3, 1), cam2lidar(3, 2), cam2lidar(3, 3));
        //     // ros::shutdown();
        //     std::cout << "camera.projectionM" << std::endl;
        //     std::cout << camera.projectionM << std::endl;
        //     transform.lidar2imageM = camera.projectionM * temp;
        //     std::cout << "transform.lidar2imageM" << std::endl;
        //     std::cout << transform.lidar2imageM << std::endl;
        // }

        pcl::fromROSMsg(*pc, *cloud_from_lidar);
        if (flag.USE_NUSCENES)
            pcl_ros::transformPointCloud(camera.FRAME_ID, *cloud_from_lidar, *cloud_from_lidar, tf_listener);
    }

    void Projector::pointcloudImageFilter(PointCloud::Ptr cloud)
    {
        PointCloud::Ptr temp_cloud(new PointCloud);

        for (auto point : cloud->points)
        {
            cv::Point pt = pointcloud2image(point, transform.lidar2imageM);

            //像素位置过滤
            if (pt.x > 0 && pt.x <= camera.IMAGE_WIDTH && pt.y > 0 && pt.y <= camera.IMAGE_HEIGHT)
                temp_cloud->points.push_back(point);
        }

        cloud_in_image = temp_cloud;
    }

    void Projector::pointcloudYOLOV5BoundingBoxFilter(PointCloud::Ptr cloud)
    {
        for (auto point : cloud->points)
        {
            cv::Point pt = pointcloud2image(point, transform.lidar2imageM);
            for (auto &target : yolov5_targets)
            {
                if (pt.x > target.boundingbox.xmin && pt.x < target.boundingbox.xmax && pt.y > target.boundingbox.ymin && pt.y < target.boundingbox.ymax)
                    target.points.push_back(point);
            }
        }
    }

    void Projector::pointcloudGroundFilter(PointCloud::Ptr cloud)
    {
    }

    // void Projector::virtualPointCloudGenerate(cv::Mat image, std::vector<Point> real_pc, std::vector<Point> vitual_pc)
    // {
    // }

    void Projector::pointcloudEuclideanClusterForYOLOV5()
    {
        for (auto &target : yolov5_targets)
        {
            std::vector<pcl::PointIndices> cluster_indices;
            cluster_indices = pointcloudEuclideanCluster(target.pc_Ptr, lidar.ec_cluster_params.cluster_tolerance, lidar.ec_cluster_params.min_cluster_size, lidar.ec_cluster_params.max_cluster_size);

            // 选择聚类后最大的一块
            int max_cluster_size = 0;
            std::vector<int> max_cluster;
            for (auto cluster_index : cluster_indices)
            {
                int cluster_size = cluster_index.indices.size();
                if (cluster_size > max_cluster_size)
                {
                    max_cluster_size = cluster_size;
                    max_cluster = cluster_index.indices;
                }
            }

            PointCloud::Ptr cloud_cluster(new PointCloud);
            for (auto it : max_cluster)
                cloud_cluster->points.push_back(target.pc_Ptr->points[it]);
            target.pc_Ptr = cloud_cluster;
        }
    }

    void Projector::pointcloudEuclideanClusterFor3D()
    {
        std::vector<ECTarget>().swap(ec_targets);
        auto cluster_indices = pointcloudEuclideanCluster(cloud_in_image, lidar.ec_cluster_params.cluster_tolerance, lidar.ec_cluster_params.min_cluster_size, lidar.ec_cluster_params.max_cluster_size);
        for (auto cluster_index : cluster_indices)
        {
            ECTarget target;
            PointCloud::Ptr cloud_cluster(new PointCloud);
            for (auto it : cluster_index.indices)
                cloud_cluster->points.push_back(cloud_in_image->points[it]);
            target.pc_Ptr = cloud_cluster;
            pointcloudWeightCenterPositionCalculation(target.pc_Ptr, target.center);
            pointcloudAABBPositionCalculation(target.pc_Ptr, target.min_point_AABB, target.max_point_AABB);
            ec_targets.push_back(target);
        }
    }

    void Projector::drawPoint(cv::Mat &img, PointType point)
    {
        cv::Point pt = pointcloud2image(point, transform.lidar2imageM);
        float depth = euclideanDistance(point);

        float H = std::min(360.0, 360.0 * depth / lidar.max_distance);
        float V = 1;
        float S = 1;

        cv::Scalar color = HSV2RGB(cv::Scalar(H, S, V));
        cv::circle(img, pt, 3, color, -1);
    }

    void Projector::drawPictureFromPointCloud(cv::Mat &img)
    {
        for (auto point : cloud_in_image->points)
            drawPoint(img, point);
    }

    void Projector::drawPictureFromYOLOV5(cv::Mat &img)
    {
        for (auto target : yolov5_targets)
        {
            auto box = target.boundingbox;
            auto center = target.center;
            cv::rectangle(img, cv::Rect(cv::Point(box.xmax, box.ymax), cv::Point(box.xmin, box.ymin)), cv::Scalar(255, 255, 255));
            cv::putText(img, box.Class, cv::Point(box.xmin, box.ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "x:" + std::to_string(center.x), cv::Point(box.xmin, box.ymin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "y:" + std::to_string(center.y), cv::Point(box.xmin, box.ymin + 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "z:" + std::to_string(center.z), cv::Point(box.xmin, box.ymin + 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

            for (auto point : target.points)
                drawPoint(img, point);
        }
    }

    void Projector::drawPictureFrom3D(cv::Mat &img)
    {
        int green = 255, red = 255;
        for (auto target : ec_targets)
        {
            for (auto point : target.pc_Ptr->points)
            {
                cv::Point pt = pointcloud2image(point, transform.lidar2imageM);
                cv::circle(img, pt, 3, cv::Scalar(0, green, red), -1);
            }
            green -= 30, red -= 30;
        }
    }

    void Projector::cloudInImagePublish(PointCloud::Ptr cloud)
    {
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud, cloud_publish);
        cloud_publish.header.frame_id = lidar.FRAME_ID;
        cloud_in_image_pub.publish(cloud_publish);
    }

    // void Projector::boundingBoxesPositionPublish()
    // {
    //     geometry_msgs::Point position;
    //     float max_probability = 0.0;
    //     for (auto target : yolov5_targets)
    //     {
    //         if (target.boundingbox.probability > max_probability)
    //         {
    //             position.x = target.position.x;
    //             position.y = target.position.y;
    //             position.z = target.position.z;
    //             max_probability = target.boundingbox.probability;
    //         }
    //     }
    //     lidar_boundingBoxesPosition_pub.publish(position);
    // }

    void Projector::boundingBoxArrayPublish()
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar.FRAME_ID;
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.lifetime = ros::Duration(0.1);
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        int marker_id = 0;
        for (auto target : ec_targets)
        {
            marker.id = marker_id;
            marker.color.r = 0.1 * marker_id;
            marker.color.g = 1 - 0.1 * marker_id;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.pose.position.x = target.center.x;
            marker.pose.position.y = target.center.y;
            marker.pose.position.z = target.center.z;
            marker.scale.x = target.max_point_AABB.x - target.min_point_AABB.x;
            marker.scale.y = target.max_point_AABB.y - target.min_point_AABB.y;
            marker.scale.z = target.max_point_AABB.z - target.min_point_AABB.z;
            marker_array.markers.push_back(marker);
            ++marker_id;
        }
        lidar_boundingBoxesArray_pub.publish(marker_array);
    }

    // void Projector::boundingBoxArrayPublish()
    // {
    //     jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
    //     boundingbox_array.header.frame_id = lidar.FRAME_ID;
    //     int i = 0;
    //     for (auto target : yolov5_targets)
    //     {
    //         jsk_recognition_msgs::BoundingBox bb;
    //         bb.header.frame_id = lidar.FRAME_ID;
    //         bb.header.stamp = ros::Time();
    //         bb.value = 0;
    //         bb.label = i;

    //         bb.pose.position.x = target.position.x;
    //         bb.pose.position.y = target.position.y;
    //         bb.pose.position.z = target.position.z;
    //         bb.pose.orientation.x = 0.0;
    //         bb.pose.orientation.y = 0.0;
    //         bb.pose.orientation.z = 0.0;
    //         bb.pose.orientation.w = 1.0;
    //         bb.dimensions.x = target.max_point_AABB.x - target.min_point_AABB.x;
    //         bb.dimensions.y = target.max_point_AABB.y - target.min_point_AABB.y;
    //         bb.dimensions.z = target.max_point_AABB.z - target.min_point_AABB.z;

    //         boundingbox_array.boxes.push_back(bb);
    //         i++;
    //     }
    //     lidar_boundingBoxesArray_pub.publish(boundingbox_array);
    // }

    void Projector::projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        imageCallback(img);
        pointcloudCallback(pc);

        pointcloudPassThroughFilter(cloud_from_lidar, cloud_from_lidar,
                                    lidar.pc_region.xmin, lidar.pc_region.xmax,
                                    lidar.pc_region.ymin, lidar.pc_region.ymax,
                                    lidar.pc_region.zmin, lidar.pc_region.zmax);
        pointcloudImageFilter(cloud_from_lidar);
        switch (mode)
        {
        case 0: // vinilla
            drawPictureFromPointCloud(undistort_img);
            break;
        case 1: // yolov5
            pointcloudYOLOV5BoundingBoxFilter(cloud_in_image);
            pointcloudEuclideanClusterForYOLOV5();
            for (auto &target : yolov5_targets)
                pointcloudWeightCenterPositionCalculation(target.points, target.center);
            drawPictureFromYOLOV5(undistort_img);
            break;
        case 2: // euclidean cluster
            pointcloudEuclideanClusterFor3D();
            boundingBoxArrayPublish();
            drawPictureFrom3D(undistort_img);
            break;
        case 3: // virtual point
            // pointcloudGroundFilter(cloud_from_lidar); // 地面点过滤
            // virtualPointCloudGenerate(undistort_img, real_pointcloud, virtual_pointcloud);
            break;
        default:
            ROS_ERROR("Wrong mode setting");
        }

        projected_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistort_img).toImageMsg());
        cloudInImagePublish(cloud_in_image);

        cv::imshow("projection", undistort_img);
        cv::waitKey(1);
    }

    void Projector::yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes)
    {
        //清空yolov5_targets
        std::vector<YOLOV5Target>().swap(yolov5_targets);

        auto temp = boxes->bounding_boxes;
        for (auto t : temp)
        {
            YOLOV5Target target;
            target.boundingbox.num = t.num;
            target.boundingbox.probability = t.probability;
            target.boundingbox.xmax = t.xmax;
            target.boundingbox.xmin = t.xmin;
            target.boundingbox.ymax = t.ymax;
            target.boundingbox.ymin = t.ymin;
            target.boundingbox.Class = t.Class;
            yolov5_targets.push_back(target);
        }
    }

    Projector::Projector() : nh("~")
    {
        initParamsFromYAML();
        initClassMember();

        //消息同步
        message_filters::Subscriber<sensor_msgs::Image>
            img_sub(nh, camera.TOPIC, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, lidar.TOPIC, 5);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), img_sub, pcl_sub);
        sync.registerCallback(boost::bind(&Projector::projectionCallback, this, _1, _2));

        if (mode == 1)
            yolov5_boundingBoxes_sub = nh.subscribe<yolov5_ros_msgs::BoundingBoxes>("/yolov5/BoundingBoxes", 10, &Projector::yolov5Callback, this);

        image_transport::ImageTransport imageTransport(nh);
        projected_image_pub = imageTransport.advertise("/projector/projected_image", 1);
        cloud_in_image_pub = nh.advertise<sensor_msgs::PointCloud2>("/projector/cloud_in_image", 1);
        lidar_boundingBoxesPosition_pub = nh.advertise<geometry_msgs::Point>("/projector/position", 1);
        lidar_boundingBoxesArray_pub = nh.advertise<visualization_msgs::MarkerArray>("/projector/bounding_boxes", 1);

        ROS_INFO("Projector init completely.");

        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_camera_projection");
    Projection::Projector Projector;
    return 0;
}