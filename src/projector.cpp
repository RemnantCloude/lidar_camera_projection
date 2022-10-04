/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-10-04 17:21:27
 * @FilePath: /test_ws/src/lidar_camera_projection/src/projector.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/pointcloud_process.h"
#include "lidar_camera_projection/projector.h"
#include "lidar_camera_projection/algorithm.h"

#include <Eigen/Core>
#include <Eigen/Dense>

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
        nh.param<bool>("flag/show_image", flag.SHOW_IMAGE, true);
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
        nh.param<int>("lidar/rings", lidar.RINGS, 32);
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
        ROS_INFO("Mode:%d", mode);
    }

    void Projector::initClassMember()
    {
        cloud_from_lidar = boost::make_shared<PointCloud>();
        cloud_in_image = boost::make_shared<PointCloud>();
    }

    void Projector::refreshTargets()
    {
        std::vector<Target>().swap(targets);
    }

    void
    Projector::imageCallback(const sensor_msgs::Image::ConstPtr &img)
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
        if (flag.USE_NUSCENES)
        {
            sensor_msgs::PointCloud2 pointcloud;
            pcl_ros::transformPointCloud(camera.FRAME_ID, *pc, pointcloud, tf_listener);
            pcl::fromROSMsg(pointcloud, *cloud_from_lidar);
        }
        else
        {
            pcl::fromROSMsg(*pc, *cloud_from_lidar);
        }
        // if (flag.USE_NUSCENES)
        // {
        //     pcl_ros::transformPointCloud(camera.FRAME_ID, *cloud_from_lidar, *cloud_from_lidar, tf_listener);
        //     tf::StampedTransform transform_lidar2cam;

        //     tf_listener.lookupTransform("/lidar_top", "/cam_front",
        //                                 ros::Time(0), transform_lidar2cam);
        //     Eigen::Matrix4f lidar2cam;
        //     pcl_ros::transformAsMatrix(transform_lidar2cam, lidar2cam);
        //     pcl::transformPointCloud(*cloud_from_lidar, *cloud_from_lidar, lidar2cam);
        // }
    }

    void Projector::pointcloudImageFilter(PointCloud::Ptr cloud)
    {
        PointCloud::Ptr temp_cloud(new PointCloud);

        for (auto point : cloud->points)
        {
            cv::Point pt = point2pixel(point, transform.lidar2imageM);

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
            cv::Point pt = point2pixel(point, transform.lidar2imageM);
            for (auto &target : targets)
            {
                if (pt.x > target.boundingbox.xmin && pt.x < target.boundingbox.xmax && pt.y > target.boundingbox.ymin && pt.y < target.boundingbox.ymax)
                    target.pc_Ptr->points.push_back(point);
            }
        }
    }

    void Projector::pointcloudGroundFilter(PointCloud::Ptr cloud)
    {
    }

    // void Projector::virtualPointCloudGenerate(cv::Mat image, std::vector<Point> real_pc, std::vector<Point> vitual_pc)
    // {
    // }

    void Projector::pointcloudEuclideanClusterForYOLOV5(std::vector<Target> &targets)
    {
        for (auto &target : targets)
        {
            auto cluster_indices = pointcloudEuclideanCluster(
                target.pc_Ptr,
                lidar.ec_cluster_params.cluster_tolerance,
                lidar.ec_cluster_params.min_cluster_size,
                lidar.ec_cluster_params.max_cluster_size);

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

    void Projector::pointcloudEuclideanClusterFor3D(PointCloud::Ptr cloud)
    {
        refreshTargets();

        auto cluster_indices = pointcloudEuclideanCluster(
            cloud,
            lidar.ec_cluster_params.cluster_tolerance,
            lidar.ec_cluster_params.min_cluster_size,
            lidar.ec_cluster_params.max_cluster_size);
        for (auto cluster_index : cluster_indices)
        {
            Target target;
            PointCloud::Ptr cloud_cluster(new PointCloud);
            for (auto it : cluster_index.indices)
                cloud_cluster->points.push_back(cloud->points[it]);
            target.pc_Ptr = cloud_cluster;
            // pointcloudWeightCenterPositionCalculation(target.pc_Ptr, target.center);
            // pointcloudAABBPositionCalculation(target.pc_Ptr, target.min_point_AABB, target.max_point_AABB);
            targets.push_back(target);
        }
    }

    void Projector::pointcloudRingCluster(PointCloud::Ptr cloud)
    {
    }

    void Projector::imageBoundingBoxCalculation(std::vector<Target> &targets)
    {
        for (auto &target : targets)
        {
            std::vector<cv::Point> pts;
            for (auto point : target.pc_Ptr->points)
            {
                cv::Point pt = point2pixel(point, transform.lidar2imageM);
                pts.push_back(pt);
            }
            cv::Point min, max;
            minRect(pts, min, max);
            target.boundingbox.xmin = min.x;
            target.boundingbox.ymin = min.y;
            target.boundingbox.xmax = max.x;
            target.boundingbox.ymax = max.y;
        }
    }

    void Projector::drawPoint(cv::Mat &img, PointType point)
    {
        cv::Point pt = point2pixel(point, transform.lidar2imageM);
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
        for (auto target : targets)
        {
            auto box = target.boundingbox;
            auto center = target.center;
            cv::rectangle(img, cv::Rect(cv::Point(box.xmax, box.ymax), cv::Point(box.xmin, box.ymin)), cv::Scalar(255, 255, 255));
            cv::putText(img, box.Class, cv::Point(box.xmin, box.ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "x:" + std::to_string(center.x), cv::Point(box.xmin, box.ymin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "y:" + std::to_string(center.y), cv::Point(box.xmin, box.ymin + 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "z:" + std::to_string(center.z), cv::Point(box.xmin, box.ymin + 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

            for (auto point : target.pc_Ptr->points)
                drawPoint(img, point);
        }
    }

    void Projector::drawPictureFrom3D(cv::Mat &img)
    {
        for (auto target : targets)
        {
            auto box = target.boundingbox;
            cv::rectangle(img, cv::Rect(cv::Point(box.xmax, box.ymax), cv::Point(box.xmin, box.ymin)), cv::Scalar(255, 255, 255));
            for (auto point : target.pc_Ptr->points)
                drawPoint(img, point);
        }
    }

    void Projector::cloudInImagePublish(PointCloud::Ptr cloud)
    {
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud, cloud_publish);
        if (flag.USE_NUSCENES)
            cloud_publish.header.frame_id = camera.FRAME_ID;
        else
            cloud_publish.header.frame_id = lidar.FRAME_ID;
        cloud_in_image_pub.publish(cloud_publish);
    }

    void Projector::projectedImagePublish(cv::Mat img)
    {
        projected_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
    }

    // void Projector::boundingBoxesPositionPublish()
    // {
    //     geometry_msgs::Point position;
    //     float max_probability = 0.0;
    //     for (auto target : targets)
    //     {
    //         if (target.boundingbox.probability > max_probability)
    //         {
    //             position.x = target.position.x;
    //             position.y = target.position.y;
    //             position.z = target.position.z;
    //             max_probability = target.boundingbox.probability;
    //         }
    //     }
    //     pointcloud_boundingBoxPosition_pub.publish(position);
    // }

    void Projector::pointcloudBoundingBoxArrayPublish()
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
        for (auto target : targets)
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
        pointcloud_boundingBoxArray_pub.publish(marker_array);
    }

    void Projector::imageBoundingBoxArrayPublish()
    {
        yolov5_ros_msgs::BoundingBoxes boxes;
        boxes.header.frame_id = camera.TOPIC;
        boxes.header.stamp = ros::Time::now();

        for (auto target : targets)
        {
            yolov5_ros_msgs::BoundingBox box;
            box.xmin = target.boundingbox.xmin;
            box.xmax = target.boundingbox.xmax;
            box.ymin = target.boundingbox.ymin;
            box.ymax = target.boundingbox.ymax;
            boxes.bounding_boxes.push_back(box);
        }
        image_boundingBoxes_pub.publish(boxes);
    }

    // void Projector::pointcloudBoundingBoxArrayPublish()
    // {
    //     jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
    //     boundingbox_array.header.frame_id = lidar.FRAME_ID;
    //     int i = 0;
    //     for (auto target : targets)
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
    //     pointcloud_boundingBoxArray_pub.publish(boundingbox_array);
    // }

    void Projector::projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        imageCallback(img);
        pointcloudCallback(pc);

        pointcloudPassThroughFilter(
            cloud_from_lidar, cloud_from_lidar,
            lidar.pc_region.xmin, lidar.pc_region.xmax,
            lidar.pc_region.ymin, lidar.pc_region.ymax,
            lidar.pc_region.zmin, lidar.pc_region.zmax);
        pointcloudImageFilter(cloud_from_lidar);
        switch (mode)
        {
        case 0: // vinilla
        {
            drawPictureFromPointCloud(undistort_img);
            break;
        }
        case 1: // yolov5
        {
            pointcloudYOLOV5BoundingBoxFilter(cloud_in_image);
            pointcloudEuclideanClusterForYOLOV5(targets);
            for (auto &target : targets)
                pointcloudWeightCenterPositionCalculation(target.pc_Ptr, target.center);
            drawPictureFromYOLOV5(undistort_img);
            break;
        }
        case 2: // euclidean cluster
        {
            pointcloudEuclideanClusterFor3D(cloud_in_image);
            imageBoundingBoxCalculation(targets);
            // pointcloudBoundingBoxArrayPublish();
            imageBoundingBoxArrayPublish();
            drawPictureFrom3D(undistort_img);
            break;
        }
        case 3: // virtual point
        {
            pointcloudGroundFilter(cloud_in_image);
            pointcloudRingCluster(cloud_in_image);
            // virtualPointCloudGenerate(undistort_img, real_pointcloud, virtual_pointcloud);
            break;
        }
        default:
        {
            ROS_ERROR("Wrong mode setting");
        }
        }

        projectedImagePublish(undistort_img);
        cloudInImagePublish(cloud_in_image);

        if (flag.SHOW_IMAGE)
        {
            cv::imshow("projection", undistort_img);
            cv::waitKey(1);
        }
    }

    void Projector::yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes)
    {
        refreshTargets();

        auto temp = boxes->bounding_boxes;
        for (auto t : temp)
        {
            Target target;
            target.boundingbox.num = t.num;
            target.boundingbox.probability = t.probability;
            target.boundingbox.xmax = t.xmax;
            target.boundingbox.xmin = t.xmin;
            target.boundingbox.ymax = t.ymax;
            target.boundingbox.ymin = t.ymin;
            target.boundingbox.Class = t.Class;
            targets.push_back(target);
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
        pointcloud_boundingBoxPosition_pub = nh.advertise<geometry_msgs::Point>("/projector/position", 1);
        pointcloud_boundingBoxArray_pub = nh.advertise<visualization_msgs::MarkerArray>("/projector/pointcloud_boundingbox_array", 1);
        image_boundingBoxes_pub = nh.advertise<yolov5_ros_msgs::BoundingBoxes>("/projector/image_boundingbox_array", 1);

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