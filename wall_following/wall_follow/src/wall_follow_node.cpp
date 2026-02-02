#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

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
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/wall_follow_markers", 10);

        // 파라미터 선언 및 초기값 설정
        this->declare_parameter("kp", 1.8);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.3);
        this->declare_parameter("dis",0.8);
        this->declare_parameter("big_angle",90.0);
        this->declare_parameter("small_angle",50.0);
        // 초기값 불러오기
        kp = this->get_parameter("kp").as_double();
        ki = this->get_parameter("ki").as_double();
        kd = this->get_parameter("kd").as_double();
        dis=this->get_parameter("dis").as_double();
        big_angle=this->get_parameter("big_angle").as_double();
        small_angle=this->get_parameter("small_angle").as_double();
    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double kp;
    double ki;
    double kd;
    double dis;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double angle_min=0.0;
    double big_angle;
    double small_angle;
    // range[0]: first(angle_min)
    // range[1]: angle_min+angle_increment 
    // target angle-angle min/angle_increment => idx
    double angle_increment=0.0;
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string error_topic = "/wall_error";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

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
        double angle_b = (big_angle * M_PI) / 180.0;
        double angle_a = (small_angle * M_PI) / 180.0;
        double a = get_range(range_data,angle_a);
        double b = get_range(range_data,angle_b);
        double theta = angle_b-angle_a;
        double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
        
        double ab=b*cos(alpha);
        // define L = 1.0;
        double l=0.7;
        double cd=ab+l*sin(alpha);
        //RCLCPP_INFO(this->get_logger(), "a(50deg): %.2f, b(90deg): %.2f, alpha: %.3f", a, b, alpha);
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
        if (dt < 0.01) dt = 0.01;
        integral+=error*dt;
        double derivative=(error-prev_error)/dt;
        
        angle=kp*error+kd*derivative+ki*integral;
        double max_speed = 3;
        double min_speed = 0.8;
        double speed = max_speed - 1.5 * fabs(angle);
        speed = std::clamp(speed, min_speed, max_speed);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        
        drive_msg.drive.steering_angle = -angle; drive_msg.drive.speed=speed;
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
        this->get_parameter("dis",dis);
        this->get_parameter("big_angle",big_angle);
        this->get_parameter("small_angle",small_angle);
        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // fix velocity to implement easy
        velocity=2.0;
        this->angle_min = scan_msg->angle_min;
        this->angle_increment = scan_msg->angle_increment;

        const auto ranges=scan_msg->ranges;
        // 1.0 is desired gap distance between wall and car
        double front = get_range(const_cast<float*>(ranges.data()), 0.0);
        double left  = get_range(const_cast<float*>(ranges.data()), M_PI / 2);
        // front: 정면 거리, dis: 유지하고 싶은 벽 거리. 정면 벽이 목표보다 가까우면 직진 시 박음
        // 거기다가 좌측벽이 정면 벽과 거의 같은 거리라면 일단 유턴 or 정지 or 우회전임. 우선 확인 위해 우회전 
        
        if(front<dis+0.4&&left<front+0.3){
            pid_control(0.5, 1.0); // 우회전 코너 진입하면 강제 우회전 
            //RCLCPP_INFO(this->get_logger(), "turn right");
            return;
        }

        error=get_error(const_cast<float*>(ranges.data()),dis);
        //RCLCPP_INFO(this->get_logger(), "error: %f",error);
        error = std::clamp(error, -0.8, 0.8);
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
