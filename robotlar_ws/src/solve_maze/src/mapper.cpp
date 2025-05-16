#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <cmath>

const int MAP_WIDTH = 600;
const int MAP_HEIGHT = 600;
const double RESOLUTION = 20.0;
const cv::Scalar POINT_COLOR = cv::Scalar(255, 255, 255);

cv::Mat map_image;

tf::TransformListener *listener_ptr = nullptr;

// /scan mesajını işleyen ve lazerleri kullanan callback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float range = scan_msg->ranges[i];
        if (range < scan_msg->range_min || range > scan_msg->range_max)
            continue;

        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double x_base = range * cos(angle);
        double y_base = range * sin(angle);

        tf::Stamped<tf::Point> point_in_base(
            tf::Point(x_base, y_base, 0.0),
            scan_msg->header.stamp,
            scan_msg->header.frame_id);

        tf::Stamped<tf::Point> point_in_odom;
        try
        {
            listener_ptr->waitForTransform("odom", scan_msg->header.frame_id,
                                           scan_msg->header.stamp, ros::Duration(0.1));
            listener_ptr->transformPoint("odom", point_in_base, point_in_odom);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Dönüşüm Hatası: %s", ex.what());
            continue;
        }

        double x_odom = point_in_odom.getX();
        double y_odom = point_in_odom.getY();

        int pixel_x = static_cast<int>(MAP_WIDTH / 2 + x_odom * RESOLUTION);
        int pixel_y = static_cast<int>(MAP_HEIGHT / 2 - y_odom * RESOLUTION);

        if (pixel_x >= 0 && pixel_x < MAP_WIDTH && pixel_y >= 0 && pixel_y < MAP_HEIGHT)
        {
            cv::circle(map_image, cv::Point(pixel_x, pixel_y), 1, POINT_COLOR, -1);
        }
    }

    cv::imshow("Robot Haritalama", map_image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mapper");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    listener_ptr = &listener;

    cv::namedWindow("Robot Haritalama", cv::WINDOW_AUTOSIZE);
    map_image = cv::Mat::ones(MAP_HEIGHT, MAP_WIDTH, CV_8UC3) * 255;
    map_image.setTo(cv::Scalar(0, 0, 0));
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
