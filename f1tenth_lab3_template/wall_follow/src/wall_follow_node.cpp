#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
using namespace std;
using std::placeholders::_1; // first param place
class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>(error_topic, 10);
        // 파라미터 선언 및 초기값 설정
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.1);

        // 초기값 불러오기
        kp = this->get_parameter("kp").as_double();
        ki = this->get_parameter("ki").as_double();
        kd = this->get_parameter("kd").as_double();
    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double kp;
    double ki;
    double kd;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double angle_min=0.0;
    // range[0]: first(angle_min)
    // range[1]: angle_min+angle_increment 
    // target angle-angle min/angle_increment => idx
    double angle_increment=0.0;
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string error_topic = "/wall_error";
    /// TODO: create ROS subscribers and publishers

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        int idx = static_cast<int>((angle - this->angle_min) / this->angle_increment);
        if(idx<0) idx=0; 
        return range_data[idx];
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        double angle_b = (90.0 * M_PI) / 180.0;
        double angle_a = (60.0 * M_PI) / 180.0;
        double a = get_range(range_data,angle_a);
        double b = get_range(range_data,angle_b);
        double theta = angle_b-angle_a;
        double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
        double ab=b*cos(alpha);
        // define L = 1.0;
        double l=0.7;
        double cd=ab+l*sin(alpha);
        RCLCPP_INFO(this->get_logger(), "a(50deg): %.2f, b(90deg): %.2f, alpha: %.3f", a, b, alpha);
        return dist-cd;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        static double prev_time = this->now().seconds(); // 처음 한 번만 초기화
        double cur_time=this->now().seconds();
        double dt=cur_time-prev_time;

        integral+=error*dt;
        double derivative=(error-prev_error)/dt;
        
        angle=kp*error+kd*derivative+ki*integral;


        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        
        drive_msg.drive.steering_angle = -angle; drive_msg.drive.speed=velocity;
        drive_pub_->publish(drive_msg);
        auto error_msg = std_msgs::msg::Float64();
        error_msg.data = error;
        error_pub_->publish(error_msg);

        prev_error=error;
        prev_time=cur_time;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // fix velocity to implement easy
        velocity=3.0;
        this->angle_min = scan_msg->angle_min;
        this->angle_increment = scan_msg->angle_increment;

        const auto ranges=scan_msg->ranges;
        // 1.0 is desired gap distance between wall and car
        error=get_error(const_cast<float*>(ranges.data()),1.0);
        pid_control(error,velocity);
        // TODO: actuate the car with PID

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}