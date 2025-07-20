// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <std_msgs/String.h> 

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert.h"
#include "liw/lio/lidarodom.h"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

nav_msgs::Path laserOdoPath;
nav_msgs::Path orinOdoPath;
nav_msgs::Path orinvinsOdoPath;

zjloc::lidarodom *lio;
zjloc::CloudConvert *convert;
std::mutex odoLock; //vins里程计锁

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{

    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    zjloc::common::Timer::Evaluate([&]()
                                   { 
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");
    
    zjloc::common::Timer::Evaluate([&]()
    {
        std::sort(cloud_out.begin(), cloud_out.end(), [](const point3D &a, const point3D &b) -> bool
        {
            return a.relative_time < b.relative_time;
        });
    }, "laser sort");

    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
}

void standard_pcl_cbk2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            double sample_size = lio->getIndex() < 20 ? 0.01 : 0.05;
            // double sample_size = 0.05;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
                                       "laser ds");

        zjloc::common::Timer::Evaluate([&]()
        {
            std::sort(cloud_out.begin(), cloud_out.end(), [](const point3D &a, const point3D &b) -> bool
            {
                return a.relative_time < b.relative_time;
            });
        }, "laser sort");

        // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec() - convert->getTimeSpan(), convert->getTimeSpan())); //  FIXME: for staircase dataset(header timestamp is the frame end)
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan())); //  normal
    }
    c++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
    lio->pushData(imu);
}

void compressed_image_cbk(const sensor_msgs::CompressedImageConstPtr &msg)
{
    // note 跟之前的voxelmap的使用方法不同, 直接调用cv_bridge出现问题
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    lio->pushData(img, msg->header.stamp.toSec());
    img.release();
}

void updateStatus(const std_msgs::Int32::ConstPtr &msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::string config_file;
    if (nh.getParam("config_file", config_file))
    {
        // 输出读取的 config_file 路径
        std::cout << "\033[32m" << "config_file: " << config_file << "\033[0m" << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to get param 'config_file'");
    }

    // std::string config_file = std::string(ROOT_DIR) + "config/mapping.yaml";
    // std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    lio = new zjloc::lidarodom();
    if (!lio->init(config_file))
    {
        return -1;
    }

    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser")
                pub_scan.publish(*cloud_ptr_output);
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);
    ros::Publisher puborinOdometryPath = nh.advertise<nav_msgs::Path>("/orin_odometry_path", 5);
    ros::Publisher puborinvinsPath = nh.advertise<nav_msgs::Path>("/orin_vins_path", 5);
    ros::Publisher pubLaserPose = nh.advertise<geometry_msgs::PoseStamped>("/laser_pose", 100);
    ros::Publisher puborinLaserPose = nh.advertise<geometry_msgs::PoseStamped>("/orin_laser_pose", 100);

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            if (topic_name == "laser")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

                // publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(stamp);

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry.publish(laserOdometry);

                //  publish path
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                pubLaserPose.publish(laserPose);
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath.publish(laserOdoPath);
            }

            if (topic_name == "orin_laser")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "orin_base_link"));

                // publish odometry
                nav_msgs::Odometry orinOdometry;
                orinOdometry.header.frame_id = "map";
                orinOdometry.child_frame_id = "orin_base_link";
                orinOdometry.header.stamp = ros::Time().fromSec(stamp);

                orinOdometry.pose.pose.orientation.x = q_current.x();
                orinOdometry.pose.pose.orientation.y = q_current.y();
                orinOdometry.pose.pose.orientation.z = q_current.z();
                orinOdometry.pose.pose.orientation.w = q_current.w();
                orinOdometry.pose.pose.position.x = pose.translation().x();
                orinOdometry.pose.pose.position.y = pose.translation().y();
                orinOdometry.pose.pose.position.z = pose.translation().z();
                // pubLaserOdometry.publish(laserOdometry);

                //  publish path
                geometry_msgs::PoseStamped orinPose;
                orinPose.header = orinOdometry.header;
                orinPose.pose = orinOdometry.pose.pose;
                puborinLaserPose.publish(orinPose);
                orinOdoPath.header.stamp = orinOdometry.header.stamp;
                orinOdoPath.poses.push_back(orinPose);
                orinOdoPath.header.frame_id = "/map";
                puborinOdometryPath.publish(orinOdoPath);
            }

            if (topic_name == "orin_vins")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "orin_vins"));

                // publish odometry
                nav_msgs::Odometry orinvinsOdometry;
                orinvinsOdometry.header.frame_id = "map";
                orinvinsOdometry.child_frame_id = "orin_vins";
                orinvinsOdometry.header.stamp = ros::Time().fromSec(stamp);

                orinvinsOdometry.pose.pose.orientation.x = q_current.x();
                orinvinsOdometry.pose.pose.orientation.y = q_current.y();
                orinvinsOdometry.pose.pose.orientation.z = q_current.z();
                orinvinsOdometry.pose.pose.orientation.w = q_current.w();
                orinvinsOdometry.pose.pose.position.x = pose.translation().x();
                orinvinsOdometry.pose.pose.position.y = pose.translation().y();
                orinvinsOdometry.pose.pose.position.z = pose.translation().z();
                // pubLaserOdometry.publish(laserOdometry);

                //  publish path
                geometry_msgs::PoseStamped orinvinsPose;
                orinvinsPose.header = orinvinsOdometry.header;
                orinvinsPose.pose = orinvinsOdometry.pose.pose;
                orinvinsOdoPath.header.stamp = orinvinsOdometry.header.stamp;
                orinvinsOdoPath.poses.push_back(orinvinsPose);
                orinvinsOdoPath.header.frame_id = "/map";
                puborinvinsPath.publish(orinvinsOdoPath);
            }

            return true;
        }

    );

    ros::Publisher image_pub = nh.advertise<sensor_msgs::CompressedImage>("/img", 10);
    auto image_pub_func = std::function<bool(std::string& topic_name, cv::Mat image, double time)>(
        [&](std::string& topic_name, cv::Mat image, double time)
        {
            // 1. 构造 ROS 头
            std_msgs::Header header;
            header.stamp = ros::Time().fromSec(time);
            header.frame_id = "camera";

            // 2. 压缩 cv::Mat -> JPEG
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
            cv::imencode(".jpg", image, buffer, params);

            // 3. 填充 CompressedImage 并发布
            sensor_msgs::CompressedImagePtr msg(
            new sensor_msgs::CompressedImage());
            msg->header = header;
            msg->format = "jpeg";
            msg->data   = std::move(buffer);

            image_pub.publish(msg);
            return true;
        }
    );

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);
    ros::Publisher text_pub = nh.advertise<std_msgs::String>("/text", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, std::string time2)>(
        [&](std::string &topic_name, double time1, std::string time2)
        {
            std_msgs::Float32 time_rviz;
            std_msgs::String text_msg;

            text_msg.data = time2;
            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);
            else
                dist_pub.publish(time_rviz);

            if (topic_name == "text") text_pub.publish(text_msg);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);
    lio->setFunc(image_pub_func);

    convert = new zjloc::CloudConvert;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    std::string image_topic = yaml["common"]["img_topic"].as<std::string>();

    ros::Subscriber subLaserCloud = convert->lidar_type_ == zjloc::CloudConvert::LidarType::AVIA
                                        ? nh.subscribe(laser_topic, 100, livox_pcl_cbk)
                                        : nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk2);

    ros::Subscriber sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imuHandler);

    ros::Subscriber sub_type = nh.subscribe<std_msgs::Int32>("/change_status", 2, updateStatus);

    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(
                              "/vins/odometry/imu_propagate_ros", 
                              2000, 
                              [lio](const nav_msgs::Odometry::ConstPtr& msg){
                                lio->pushData(msg);
                              }, 
                              ros::TransportHints().tcpNoDelay());

    ros::Subscriber sub_image = nh.subscribe( image_topic, 200, compressed_image_cbk);

    std::thread measurement_process(&zjloc::lidarodom::run, lio);

    ros::spin();

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}