/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-26 21:44:30
 * @FilePath: /test_ws/src/lidar_camera_projection/src/project.cpp
 * @Description:
 *
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved.
 */

#include "lidar_camera_projection/project.h"
#include "lidar_camera_projection/algorithm.h"

#include <typeinfo>

#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace Projection
{
    void Projector::initParamsFromYAML()
    {
        nh.param<int>("mode", mode, 0);
        nh.param<bool>("flag/if_show_time", flag.IF_SHOW_TIME, true);

        nh.param<std::string>("camera/topic", camera.TOPIC, "/lbas_image");
        nh.param<int>("camera/image_width", camera.IMAGE_WIDTH, 1440);
        nh.param<int>("camera/image_height", camera.IMAGE_HEIGHT, 1080);
        nh.param<std::vector<double>>("camera/intrinsic", camera.intrinsicV, std::vector<double>());
        nh.param<std::vector<double>>("camera/projection", camera.projectionV, std::vector<double>());
        nh.param<std::vector<double>>("camera/distortion", camera.distortionV, std::vector<double>());
        camera.intrinsicC = cv::Mat(camera.intrinsicV).reshape(0, 3);   // 3*3
        camera.projectionC = cv::Mat(camera.projectionV).reshape(0, 3); // 3*4
        camera.distortionC = cv::Mat(camera.distortionV).reshape(0, 1); // 1*5

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

        nh.param<std::vector<double>>("transform/lidar2camera", transform.lidar2cameraV, std::vector<double>());
        transform.lidar2cameraC = cv::Mat(transform.lidar2cameraV).reshape(0, 4); // 4*4

        transform.lidar2imageC = camera.projectionC * transform.lidar2cameraC;

        ROS_INFO("Read initial param file successfully.");
    }

    void Projector::initClassMember()
    {
        cloud_from_lidar = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_in_image = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        // lidar.extractor.setClusterTolerance(lidar.ec_cluster_params.cluster_tolerance);
        // lidar.extractor.setMinClusterSize(lidar.ec_cluster_params.min_cluster_size);
        // lidar.extractor.setMaxClusterSize(lidar.ec_cluster_params.max_cluster_size);
        // lidar.extractor.setSearchMethod(lidar.kd_tree);
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
        cv::undistort(cv_ptr->image, undistort_img, camera.intrinsicC, camera.distortionC);
    }

    void Projector::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        pcl::fromROSMsg(*pc, *cloud_from_lidar);
    }

    void Projector::pointcloudPassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::PassThrough<pcl::PointXYZI> passthrough;
        passthrough.setKeepOrganized(false);

        passthrough.setInputCloud(cloud);
        passthrough.setFilterFieldName("x");
        passthrough.setFilterLimits(lidar.pc_region.xmin, lidar.pc_region.xmax);
        passthrough.filter(*cloud);

        passthrough.setInputCloud(cloud);
        passthrough.setFilterFieldName("y");
        passthrough.setFilterLimits(lidar.pc_region.ymin, lidar.pc_region.ymax);
        passthrough.filter(*cloud);

        passthrough.setInputCloud(cloud);
        passthrough.setFilterFieldName("z");
        passthrough.setFilterLimits(lidar.pc_region.zmin, lidar.pc_region.zmax);
        passthrough.filter(*cloud);
    }

    void Projector::pointcloudImageFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (auto point : cloud->points)
        {
            cv::Point pt = pointcloud2image(point, transform.lidar2imageC);

            //像素位置过滤
            if (pt.x > 0 || pt.x <= camera.IMAGE_WIDTH || pt.y > 0 || pt.y <= camera.IMAGE_HEIGHT)
                temp_cloud->points.push_back(point);
        }

        cloud_in_image = temp_cloud;
    }

    void Projector::pointcloudYOLOV5BoundingBoxFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        for (auto point : cloud->points)
        {
            cv::Point pt = pointcloud2image(point, transform.lidar2imageC);
            for (auto &target : yolov5_targets)
            {
                if (pt.x > target.boundingbox.xmin && pt.x < target.boundingbox.xmax && pt.y > target.boundingbox.ymin && pt.y < target.boundingbox.ymax)
                {
                    target.points.push_back(point);
                    target.pts.push_back(pt);
                }
            }
        }
    }

    void Projector::pointcloudGroundFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
    }

    // void Projector::virtualPointCloudGenerate(cv::Mat image, std::vector<Point> real_pc, std::vector<Point> vitual_pc)
    // {
    // }

    std::vector<pcl::PointIndices> Projector::pointcloudEuclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_Ptr)
    {
        std::vector<pcl::PointIndices> cluster_indices;
        if (pc_Ptr->points.empty() == true)
            return cluster_indices;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
        kd_tree->setInputCloud(pc_Ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extractor;

        extractor.setClusterTolerance(lidar.ec_cluster_params.cluster_tolerance);
        extractor.setMinClusterSize(lidar.ec_cluster_params.min_cluster_size);
        extractor.setMaxClusterSize(lidar.ec_cluster_params.max_cluster_size);
        extractor.setSearchMethod(kd_tree);
        extractor.setInputCloud(pc_Ptr);
        extractor.extract(cluster_indices);

        return cluster_indices;
    }

    void Projector::pointcloudEuclideanClusterForYOLOV5()
    {
        for (auto &target : yolov5_targets)
        {
            std::vector<pcl::PointIndices> cluster_indices;
            cluster_indices = pointcloudEuclideanCluster(target.pc_Ptr);

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

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (auto it : max_cluster)
                cloud_cluster->points.push_back(target.pc_Ptr->points[it]);
            target.pc_Ptr = cloud_cluster;
        }
    }

    cv::Mat Projector::drawPictureFromPointCloud(cv::Mat &img)
    {
        for (auto point : cloud_in_image->points)
        {
            cv::Point pt = pointcloud2image(point, transform.lidar2imageC);
            cv::circle(img, pt, 3, cv::Scalar(0, 0, 255), -1);
        }
        return img;
    }

    cv::Mat Projector::drawPictureFromYOLOV5(cv::Mat &img)
    {
        for (auto target : yolov5_targets)
        {
            auto box = target.boundingbox;
            auto position = target.position;
            cv::rectangle(img, cv::Rect(cv::Point(box.xmax, box.ymax), cv::Point(box.xmin, box.ymin)), cv::Scalar(255, 255, 255));
            cv::putText(img, box.Class, cv::Point(box.xmin, box.ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "x:" + std::to_string(position.x), cv::Point(box.xmin, box.ymin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "y:" + std::to_string(position.y), cv::Point(box.xmin, box.ymin + 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, "z:" + std::to_string(position.z), cv::Point(box.xmin, box.ymin + 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

            for (auto pt : target.pts)
            {
                // float valDenth = std::sqrt((it->x) * (it->x) + (it->y) * (it->y) + (it->z) * (it->z)); //TODO depth
                // float maxVal = 10.0;
                // int red = std::min(255, (int)it->intensity * 4);

                // int green = std::min(255, (int)(255 * (1 - abs((valDenth - maxVal) / maxVal))));
                // cv::circle(img, pt, 3, cv::Scalar(0, green, red), -1);
                cv::circle(img, pt, 3, cv::Scalar(0, 0, 255), -1);
            }
        }
        return img;
    }

    cv::Mat Projector::drawPictureFromCluster(cv::Mat &img, std::vector<pcl::PointIndices> cluster_indices)
    {
        int green = 0, red = 0;
        for (auto cluster_index : cluster_indices)
        {
            for (auto idx : cluster_index.indices)
            {
                pcl::PointXYZI point = (*cloud_in_image)[idx];
                cv::Point pt = pointcloud2image(point, transform.lidar2imageC);
                cv::circle(img, pt, 3, cv::Scalar(0, green, red), -1);
            }
            green += 10, red += 10;
        }
        return img;
    }

    void Projector::pointcloudWeightCenterPositionCalculation()
    {
        for (auto &target : yolov5_targets)
        {
            double center_x = 0;
            double center_y = 0;
            double center_z = 0;
            int count = 0;
            for (auto point : target.points)
            {
                center_x += point.x;
                center_y += point.y;
                center_z += point.z;
                count++;
            }
            target.position.x = center_x / count;
            target.position.y = center_y / count;
            target.position.z = center_z / count;
        }
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

    // void Projector::boundingBoxArrayPublish()
    // {
    //     jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
    //     boundingbox_array.header.frame_id = "rslidar";
    //     int i = 0;
    //     for (auto target : yolov5_targets)
    //     {
    //         jsk_recognition_msgs::BoundingBox bb;
    //         bb.header.frame_id = "rslidar"; // TODO
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
        cv::Mat dst;

        imageCallback(img);
        pointcloudCallback(pc);

        pointcloudPassThroughFilter(cloud_from_lidar); // 位置过滤
        pointcloudImageFilter(cloud_from_lidar);       // 像素位置过滤
        switch (mode)
        {
        case 0: // vinilla
        {
            dst = drawPictureFromPointCloud(undistort_img);
            break;
        }
        case 1: // yolov5
        {
            pointcloudYOLOV5BoundingBoxFilter(cloud_in_image);
            pointcloudEuclideanClusterForYOLOV5();
            pointcloudWeightCenterPositionCalculation();
            dst = drawPictureFromYOLOV5(undistort_img);
            break;
        }
        case 2: // euclidean cluster
        {
            auto cluster_indices = pointcloudEuclideanCluster(cloud_in_image);
            dst = drawPictureFromCluster(undistort_img, cluster_indices);
            break;
        }
        case 3: // virtual point
        {
            // pointcloudGroundFilter(cloud_from_lidar); // 地面点过滤
            // virtualPointCloudGenerate(undistort_img, real_pointcloud, virtual_pointcloud);
            break;
        }
        default:
        {
            ROS_ERROR("Wrong mode setting");
        }
        }

        projected_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg());

        cv::imshow("projection", dst);
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
        projected_image_pub = imageTransport.advertise("/projected_image", 20);
        lidar_boundingBoxesArray_pub = nh.advertise<geometry_msgs::Point>("/projector/position", 1);

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