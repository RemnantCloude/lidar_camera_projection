/*
 *@author: Cloude Remnant
 *@date: 2022-09-03
 *@description:
 */

/* Include **********************************************/
#include "lidar_camera_projection/project.h"

/* Macro Definition *************************************/

/* Global Variable **************************************/

/* Function *********************************************/
namespace Projection
{
    void Projector::initParams()
    {
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
        nh.param<double>("lidar/filter/xmin", lidar.pc_region.xmin, 0.0);
        nh.param<double>("lidar/filter/xmax", lidar.pc_region.xmax, 10.0);
        nh.param<double>("lidar/filter/ymin", lidar.pc_region.ymin, -10.0);
        nh.param<double>("lidar/filter/ymax", lidar.pc_region.ymax, 10.0);
        nh.param<double>("lidar/filter/zmin", lidar.pc_region.zmin, -10.0);
        nh.param<double>("lidar/filter/zmax", lidar.pc_region.zmax, 10.0);

        nh.param<std::vector<double>>("transform/lidar2camera", transform.lidar2cameraV, std::vector<double>());
        transform.lidar2cameraC = cv::Mat(transform.lidar2cameraV).reshape(0, 4); // 4*4

        ROS_INFO("Read initial param file successfully.");
    }

    void Projector::imageCallback(const sensor_msgs::Image::ConstPtr &img, cv::Mat &undistort_img)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // 畸变矫正
        cv::undistort(cv_ptr->image, undistort_img, camera.intrinsicC, camera.distortionC);
    }

    void Projector::pointcloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);

        for (auto point : cloud->points)
        {
            //点云位置过滤
            if (point.x > lidar.pc_region.xmax || point.x < lidar.pc_region.xmin || point.y > lidar.pc_region.ymax || point.y < lidar.pc_region.ymin || point.z > lidar.pc_region.zmax || point.z < lidar.pc_region.zmin)
                continue;

            X = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            Y = camera.projectionC * transform.lidar2cameraC * X;

            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

            //像素位置过滤
            if (pt.x < 0 || pt.x >= 1440 || pt.y < 0 || pt.y >= 1080)
                continue;

            pointcloudCluster(point, pt);
        }
    }

    void Projector::pointcloudCluster(pcl::PointXYZI point, cv::Point pt)
    {
        for (auto &target : targets)
        {
            if (pt.x > target.boundingbox.xmin && pt.x < target.boundingbox.xmax && pt.y > target.boundingbox.ymin && pt.y < target.boundingbox.ymax)
            {
                target.points.push_back(point);
                target.pts.push_back(pt);
                return;
            }
        }
    }

    cv::Mat Projector::drawPicture(cv::Mat &img)
    {
        for (auto target : targets)
        {
            auto box = target.boundingbox;
            auto position = target.position;
            cv::rectangle(img, cv::Rect(cv::Point(box.xmax, box.ymax), cv::Point(box.xmin, box.ymin)), cv::Scalar(255, 255, 255));
            cv::putText(img, box.Class, cv::Point(box.xmin, box.ymin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, std::to_string(position.x), cv::Point(box.xmin, box.ymin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, std::to_string(position.y), cv::Point(box.xmin, box.ymin + 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(img, std::to_string(position.z), cv::Point(box.xmin, box.ymin + 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

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

    void Projector::calculatePointcloudPosition()
    {
        for (auto &target : targets)
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

    void Projector::boundingBoxesPositionPublish()
    {
        geometry_msgs::Point position;
        float probability = 0.0;
        for (auto target : targets)
        {
            if (target.boundingbox.probability > probability)
            {
                position.x = target.position.x;
                position.y = target.position.y;
                position.z = target.position.z;
                probability = target.boundingbox.probability;
            }
        }
        boundingBoxesPosition_pub.publish(position);
    }

    void Projector::projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        cv::Mat undistort_img;
        imageCallback(img, undistort_img);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pc, *cloud);

        pointcloudFilter(cloud); //过滤点云并将其聚类
        calculatePointcloudPosition();
        cv::Mat dst = drawPicture(undistort_img);

        boundingBoxesPositionPublish();
        image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg());

        cv::imshow("projection", dst);
        cv::waitKey(1);
    }

    void Projector::yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes)
    {
        //清空targets
        std::vector<Target> s;
        s.swap(targets);

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
        initParams();

        //消息同步
        message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, camera.TOPIC, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, lidar.TOPIC, 5);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), img_sub, pcl_sub);
        sync.registerCallback(boost::bind(&Projector::projectionCallback, this, _1, _2));

        boundingBoxes_sub = nh.subscribe<yolov5_ros_msgs::BoundingBoxes>("/yolov5/BoundingBoxes", 10, &Projector::yolov5Callback, this);

        image_transport::ImageTransport imageTransport(nh);
        image_pub = imageTransport.advertise("/projected_image", 20);
        boundingBoxesPosition_pub = nh.advertise<geometry_msgs::Point>("/projector/position", 1);

        ROS_INFO("Projector init completely.");

        ros::spin();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_camera_projection");
    Projection::Projector Projector;
    return 0;
}