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
        std::string pkg_loc = ros::package::getPath("lidar_camera_projection");
        std::ifstream infile(pkg_loc + "/config/initial_params.txt");
        double intrinsic[9];
        double projection[12];
        double distortion[5];
        double RT[16];

        for (int i = 0; i < 9; i++)
            infile >> intrinsic[i];
        cv::Mat(3, 3, 6, &intrinsic).copyTo(params.intrinsic);

        for (int i = 0; i < 12; i++)
            infile >> projection[i];
        cv::Mat(3, 4, 6, &projection).copyTo(params.projection);

        for (int i = 0; i < 5; i++)
            infile >> distortion[i];
        cv::Mat(5, 1, 6, &distortion).copyTo(params.distortion);

        for (int i = 0; i < 16; i++)
            infile >> RT[i];
        cv::Mat(4, 4, 6, &RT).copyTo(params.RT);

        ROS_INFO("Read initial param file successfully. Located at %s", (pkg_loc + "/config/initial_params.txt").c_str());
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
        cv::undistort(cv_ptr->image, undistort_img, params.intrinsic, params.distortion);
    }

    void Projector::pointcloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);

        for (auto point:cloud->points)
        {
            //点云位置过滤
            if (point.x > pc_region.xmax || point.x < pc_region.xmin || point.y > pc_region.ymax || point.y < pc_region.ymin || point.z > pc_region.zmax || point.z < pc_region.zmin)
                continue;

            X = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            Y = params.projection * params.RT * X;

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
        for (auto &target : targets) //BUG
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

            for(auto pt : target.pts)
            {
                //float valDenth = std::sqrt((it->x) * (it->x) + (it->y) * (it->y) + (it->z) * (it->z)); //TODO depth
                //float maxVal = 10.0;
                //int red = std::min(255, (int)it->intensity * 4);

                //int green = std::min(255, (int)(255 * (1 - abs((valDenth - maxVal) / maxVal))));
                //cv::circle(img, pt, 3, cv::Scalar(0, green, red), -1);
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

    void Projector::projectionPublish(cv::Mat &img)
    {
        cv_bridge::CvImagePtr cv_ptr;
        ros::Time time = ros::Time::now();
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = time;
        cv_ptr->header.frame_id = "/project";
        cv_ptr->image = img;
        image_publisher.publish(cv_ptr->toImageMsg());

        cv::imshow("projection", img);
        cv::waitKey(1);
    }

    void Projector::projectionCallback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        cv::Mat undistort_img;
        imageCallback(img, undistort_img);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // TODO no intensity data?
        pcl::fromROSMsg(*pc, *cloud);

        pointcloudFilter(cloud); //过滤点云并将其聚类
        calculatePointcloudPosition();
        cv::Mat dst = drawPicture(undistort_img);
        //projectionPublish(dst); //BUG
        cv::imshow("projection", dst);
        cv::waitKey(1);
    }

    void Projector::yolov5Callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &boxes)
    {
        //清空targets
        std::vector<Target> s;
        s.swap(targets);

        auto temp = boxes->bounding_boxes;
        for (auto t : temp) // TODO
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

    Projector::Projector()
    {
        ros::NodeHandle nh("~");
        nh.param<std::string>("camera_topic", camera_topic, "/lbas_image");
        nh.param<std::string>("lidar_topic", lidar_topic, "/rslidar_points");
        nh.param<double>("pc_xmin", pc_region.xmin, 0.0);
        nh.param<double>("pc_xmax", pc_region.xmax, 10.0);
        nh.param<double>("pc_ymin", pc_region.ymin, -10.0);
        nh.param<double>("pc_ymax", pc_region.ymax, 10.0);
        nh.param<double>("pc_zmin", pc_region.zmin, -10.0);
        nh.param<double>("pc_zmax", pc_region.zmax, 10.0);

        initParams();

        mask = cv::Mat::zeros(1080, 1440, CV_8UC1); // TODO

        //消息同步
        message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, camera_topic, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, lidar_topic, 5);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), img_sub, pcl_sub);
        sync.registerCallback(boost::bind(&Projector::projectionCallback, this, _1, _2));

        image_transport::ImageTransport imageTransport(nh);
        image_publisher = imageTransport.advertise("/projected_image", 20);

        boundingBoxes_sub = nh.subscribe<yolov5_ros_msgs::BoundingBoxes>("/yolov5/BoundingBoxes", 10, &Projector::yolov5Callback, this);

        ROS_INFO("Projector init completely.");
        ROS_INFO("camera_topic: %s", camera_topic.c_str());
        ROS_INFO("lidar_topic: %s", lidar_topic.c_str());

        ros::spin();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_camera_projection");
    Projection::Projector Projector;
    //ros::spin();
    return 0;
}