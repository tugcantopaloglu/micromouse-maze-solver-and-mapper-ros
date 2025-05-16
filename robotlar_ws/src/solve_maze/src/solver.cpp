#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>
#include <algorithm>

// bu kod ROS ile turtlebot3'un mouse maze labirentini cozmesini sagliyor. sag duvari takip ediyoruz. uzun sure calistirdiktan sonra sonuca ulasmayi basardim. hedefe ulasabiliyor. bazi yerlerde zorlaniyor ancak yine de cozmeyi basariyor. ornegin duvara cok yakin gidiyor ve bazen koselerde sikisiyormus gibi gozukuyor. bunu cozmek icin time interval da ekledim, bu sekilde sikismiyor ancak koseleri yavas donuyor. bu kodu daha iyi hale getirmek icin bazi parametreleri degistirdim. bu parametreler robotun duvara olan mesafesini, toleransini ve kose durumunu kontrol ediyor. ayrica robotun odometrisini takip ediyoruz ve hedefe ulastiginda durmasini sagliyoruz.
ros::Publisher cmd_pub;
bool reached_goal = false;
enum CornerState
{
    NORMAL,
    RIGHT_CORNER,
    LEFT_CORNER
};
CornerState corner_state = NORMAL;

double desired_distance = 0.0; // Desired wall uzakligi (d)
double tolerance = 0.0;        // Parallel band toleransi (r)
double min_front_distance = 0.6;
double max_valid_range = 8.0;

ros::Time corner_state_start_time;
double corner_state_timeout = 3.0;

double current_x = 0.0;
double current_y = 0.0;

std::vector<double> last_right_distances;
const int distance_history_size = 5;

int stable_readings_count = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (fabs(current_x) < 1.0 && fabs(current_y) < 1.0)
    {
        reached_goal = true;
        ROS_INFO("Hedefe ulasildi!");
    }
}

double sanitizeReading(double reading)
{
    if (!std::isfinite(reading) || reading > max_valid_range)
    {
        return max_valid_range;
    }
    return reading;
}

bool detectCorner(const std::vector<double> &distances, double current_dist, bool &is_right_corner)
{
    if (distances.size() < distance_history_size)
    {
        return false;
    }

    int valid_count = 0;
    for (double d : distances)
    {
        if (d < max_valid_range - 0.1)
        {
            valid_count++;
        }
    }

    if (valid_count < 3)
    {
        return false;
    }

    double avg_prev = 0.0;
    int count = 0;
    for (size_t i = 0; i < distances.size(); i++)
    {
        if (distances[i] < max_valid_range - 0.1)
        {
            avg_prev += distances[i];
            count++;
        }
    }

    if (count == 0)
        return false;
    avg_prev /= count;

    if (current_dist >= max_valid_range - 0.1)
    {
        return false;
    }

    double diff = current_dist - avg_prev;

    if (diff > 1.5 && avg_prev < 2.0)
    {
        is_right_corner = true;
        ROS_INFO("Sag kose tespit edildi: avg_prev=%.2f, current=%.2f, diff=%.2f",
                 avg_prev, current_dist, diff);
        return true;
    }
    else if (diff < -1.5 && current_dist < 2.0)
    {
        is_right_corner = false;
        ROS_INFO("Sol kose tespit edildi: avg_prev=%.2f, current=%.2f, diff=%.2f",
                 avg_prev, current_dist, diff);
        return true;
    }

    return false;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (reached_goal)
    {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        cmd_pub.publish(stop_msg);
        return;
    }

    int num_ranges = scan->ranges.size();

    int right_idx = (int)((-M_PI / 2 - scan->angle_min) / scan->angle_increment);
    int front_idx = (int)((0.0 - scan->angle_min) / scan->angle_increment);
    int front_right_idx = (int)((-M_PI / 4 - scan->angle_min) / scan->angle_increment);
    int front_left_idx = (int)((M_PI / 4 - scan->angle_min) / scan->angle_increment);
    int left_idx = (int)((M_PI / 2 - scan->angle_min) / scan->angle_increment);
    int back_right_idx = (int)((-3 * M_PI / 4 - scan->angle_min) / scan->angle_increment);

    right_idx = std::max(0, std::min(right_idx, num_ranges - 1));
    front_idx = std::max(0, std::min(front_idx, num_ranges - 1));
    front_right_idx = std::max(0, std::min(front_right_idx, num_ranges - 1));
    front_left_idx = std::max(0, std::min(front_left_idx, num_ranges - 1));
    left_idx = std::max(0, std::min(left_idx, num_ranges - 1));
    back_right_idx = std::max(0, std::min(back_right_idx, num_ranges - 1));

    double front_dist = sanitizeReading(scan->ranges[front_idx]);
    double front_right_dist = sanitizeReading(scan->ranges[front_right_idx]);
    double front_left_dist = sanitizeReading(scan->ranges[front_left_idx]);
    double right_dist = sanitizeReading(scan->ranges[right_idx]);
    double left_dist = sanitizeReading(scan->ranges[left_idx]);
    double back_right_dist = sanitizeReading(scan->ranges[back_right_idx]);

    int window_size = 5;
    int half_window = window_size / 2;
    double sum_right = 0.0;
    int valid_count = 0;

    for (int i = right_idx - half_window; i <= right_idx + half_window; i++)
    {
        if (i >= 0 && i < num_ranges)
        {
            double reading = sanitizeReading(scan->ranges[i]);
            if (reading < max_valid_range - 0.1)
            {
                sum_right += reading;
                valid_count++;
            }
        }
    }

    double avg_right = (valid_count > 0) ? (sum_right / valid_count) : max_valid_range;

    if (valid_count > 0)
    {
        last_right_distances.push_back(avg_right);
        if (last_right_distances.size() > distance_history_size)
        {
            last_right_distances.erase(last_right_distances.begin());
        }
    }

    if (corner_state != NORMAL &&
        (ros::Time::now() - corner_state_start_time).toSec() > corner_state_timeout)
    {
        ROS_INFO("Kose durumundan cikiliyor, zaman asimi");
        corner_state = NORMAL;
        last_right_distances.clear();
    }

    geometry_msgs::Twist move_cmd;

    double min_front = std::min(front_dist, std::min(front_right_dist, front_left_dist));
    if (min_front < min_front_distance)
    {
        ROS_INFO("On engel tespit edildi %.2f metre, sola donuluyor.", min_front);
        move_cmd.linear.x = 0.05;
        move_cmd.angular.z = 0.8;
        cmd_pub.publish(move_cmd);
        return;
    }

    if (corner_state == NORMAL)
    {
        bool is_right_corner;
        if (detectCorner(last_right_distances, avg_right, is_right_corner))
        {
            corner_state_start_time = ros::Time::now();
            if (is_right_corner)
            {
                corner_state = RIGHT_CORNER;
                ROS_INFO("Sag kose (90) tespit edildi. Donus basliyor.");
            }
            else
            {
                corner_state = LEFT_CORNER;
                ROS_INFO("Sol kose (270) tespit edildi. Donus basliyor.");
            }
        }
    }

    if (corner_state == NORMAL && fabs(desired_distance - avg_right) <= tolerance)
    {
        stable_readings_count++;
        if (stable_readings_count % 20 == 0)
        {
            ROS_INFO("Duvar takibi stabil %d olcum", stable_readings_count);
        }
    }
    else
    {
        stable_readings_count = 0;
    }

    // duvar takibi
    switch (corner_state)
    {
    case RIGHT_CORNER:
        ROS_INFO("Sag kosede donuluyor. Mevcut sag mesafe: %.2f", avg_right);
        move_cmd.linear.x = 0.05;
        move_cmd.angular.z = 0.5;

        if (front_right_dist < desired_distance + tolerance &&
            avg_right < max_valid_range - 0.5)
        {
            ROS_INFO("Kose donusu tamamlandi, normal duvar takibine donuluyor.");
            corner_state = NORMAL;
            last_right_distances.clear();
        }
        break;

    case LEFT_CORNER:
        ROS_INFO("Sol kosede donuluyor. Mevcut sag mesafe: %.2f", avg_right);
        move_cmd.linear.x = 0.1;
        move_cmd.angular.z = -0.3;

        if (avg_right <= (desired_distance + tolerance) && avg_right < max_valid_range - 0.5)
        {
            ROS_INFO("Kose donusu tamamlandi, normal duvar takibine donuluyor.");
            corner_state = NORMAL;
            last_right_distances.clear();
        }
        break;

    case NORMAL:
    default:
        if (avg_right >= max_valid_range - 0.1)
        {
            ROS_INFO("Sagda duvar tespit edilmedi, duvar bulmak icin saga donuluyor");
            move_cmd.linear.x = 0.1;
            move_cmd.angular.z = -0.3;
        }
        else
        {
            double error = desired_distance - avg_right;

            double k_p = 0.8;
            double k_d = 0.5;

            static double prev_error = 0;
            double error_change = error - prev_error;
            prev_error = error;

            move_cmd.angular.z = k_p * error + k_d * error_change;

            move_cmd.angular.z = std::max(-0.7, std::min(0.7, move_cmd.angular.z));

            if (fabs(error) > tolerance)
            {
                move_cmd.linear.x = 0.12;
            }
            else
            {
                move_cmd.linear.x = 0.2;
            }

            if (front_dist < 1.0)
            {
                move_cmd.linear.x *= (front_dist / 1.0);
            }

            ROS_INFO("Normal mod: Sag mesafe: %.2f, Hata: %.2f, Komut: (lin=%.2f, ang=%.2f)",
                     avg_right, error, move_cmd.linear.x, move_cmd.angular.z);
        }
        break;
    }

    cmd_pub.publish(move_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_solver");
    ros::NodeHandle nh;

    nh.param("desired_distance", desired_distance, 0.6);
    nh.param("tolerance", tolerance, 0.1);
    nh.param("min_front_distance", min_front_distance, 0.45);
    nh.param("max_valid_range", max_valid_range, 12.0);
    nh.param("corner_state_timeout", corner_state_timeout, 6.0);

    ROS_INFO("Duvar takip baslatiliyor: desired_distance=%.2f, tolerance=%.2f, min_front_distance=%.2f",
             desired_distance, tolerance, min_front_distance);

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    corner_state_start_time = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok() && !reached_goal)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist stop;
    stop.linear.x = 0;
    stop.angular.z = 0;
    cmd_pub.publish(stop);
    ROS_INFO("Gorev tamamlandi. Robot durduruldu.");

    return 0;
}