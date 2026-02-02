#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
/// CHECK: include needed ROS msg type headers and libraries
using std::placeholders::_1; // first param place
using namespace std;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_topic, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_point", 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string laser_topic ="/scan_debug";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    double threshold=2.5;
    double rb=0.18; // car width:0.22
    double disparityT=0.3;
    double safe_dist=1.0;
    double angle_min=0.0;
    double angle_increment=0.0;
    double max_speed = 6.0;
double min_speed = 1.5;
    double k_angle = 2.0;     // 조향 민감도
    double front_ref = 2.0;   // 기준 전방 거리 (m)
    double emergency_dist = 1.2;   // 이보다 가까우면 중앙 금지

    // smoothing 목적 
    double prev_angle=0.0;
    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        for(int i=0; i<1080; i++){
            if(ranges[i]>this->threshold) ranges[i]=this->threshold;
        }
        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        int max_len = 0;
        int max_st = 540; 
        int max_ed = 540;
        
        int cur_st = -1;
        int cur_len = 0;

        int start_idx = 300; 
        int end_idx = 780;

        //RCLCPP_INFO(this->get_logger(), "--- Gap Search Start ---");

        for(int i = start_idx; i < end_idx; i++){
            // gap으로 보려면 일단 거기까지 거리가 내가 주행 가능한 수준 이상은 되어야 함 
            if(ranges[i] >= safe_dist){ 
                if(cur_len == 0) cur_st = i;
                cur_len++;
            }
            else {
                if(cur_len > 0){
                    // 후보 Gap 정보 출력
                // RCLCPP_INFO(this->get_logger(), "Candidate Gap: st=%d, ed=%d, len=%d", cur_st, i-1, cur_len);
                    
                    if(cur_len > max_len){
                        max_len = cur_len;
                        max_st = cur_st;
                        max_ed = i - 1;
                    }
                    cur_len = 0;
                }
            }
        }
        
        // 마지막 구간 체크
        if(cur_len > 0){
        // RCLCPP_INFO(this->get_logger(), "Candidate Gap: st=%d, ed=%d, len=%d", cur_st, end_idx-1, cur_len);
            if(cur_len > max_len){
                max_len = cur_len;
                max_st = cur_st;
                max_ed = end_idx - 1;
            }
        }

        //RCLCPP_INFO(this->get_logger(), ">>> Final Selected: st=%d, ed=%d, len=%d", max_st, max_ed, max_len);
        
        indice[0] = max_st;
        indice[1] = max_ed;
    }
            
    void find_best_point(float* ranges, int* indice)
    {   
        int st = indice[0];
        int ed = indice[1];
        float max_val = -1.0;
        
        // 1. 해당 Gap(st ~ ed) 사이에서 가장 큰 거리값(max_val)을 찾음
        for (int i = st; i <= ed; i++) {
            if (ranges[i] > max_val) {
                max_val = ranges[i];
            }
        }

        // 2. 그 max_val을 가진 인덱스들 중 처음과 끝을 찾음
        int first_idx = -1;
        int last_idx = -1;

        for (int i = st; i <= ed; i++) {
            // float 비교이므로 아주 작은 오차 범위를 둡니다 (0.001m)
            if (std::abs(ranges[i] - max_val) < 0.001) {
                if (first_idx == -1) first_idx = i;
                last_idx = i;
            }
        }

        // 3. 동일한 최대 거리를 가진 인덱스 구간의 '중앙 인덱스' 계산
        int best_idx = (first_idx + last_idx) / 2;
        
        indice[0] = best_idx;

        // RCLCPP_INFO(this->get_logger(), "MaxDist: %.2f, BestIdx: %d (Center of indices %d~%d)", max_val, best_idx, first_idx, last_idx);
    }

    void extend_disparities(float* ranges){
        for(int i=0; i<1079; i++){
            float diff = abs(ranges[i] - ranges[i+1]);
            if(diff > disparityT){
                // 로그 추가: 어떤 인덱스에서 확장이 발생하는지 확인
                // RCLCPP_INFO(this->get_logger(), "Disparity found at idx %d, diff: %.2f", i, diff);
                
                float closer_dist = min(ranges[i], ranges[i+1]);
                // 차 반폭: rd, 장애물 거리 dist, 각도 theta=atan2(rb,d)니까 ray 개수는 theta/angle_increment
                // 근데 장애물이 멀면 더 적게 막고 가까우면 더 많이 막아야 함  
                int car_idx = static_cast<int>(
                    std::ceil( std::atan2(rb, closer_dist) / angle_increment )
                );


                if (ranges[i] < ranges[i+1]) {
                    int ed = min(i + car_idx, 1079);
                    for(int k = i + 1; k <= ed; k++){
                        if(ranges[k] > closer_dist){
                            ranges[k] = closer_dist;
                            // 확장이 일어나는 구체적인 지점 로그
                            // RCLCPP_INFO(this->get_logger(), "  -> Extending to idx %d", k);
                        }
                    }
                    // 중요: 연쇄 반응 방지를 위해 루프 인덱스를 건너뜁니다.
                    i = ed; 
                }
                else {
                    int st = max(0, i + 1 - car_idx);
                    for(int k = st; k <= i; k++){
                        if(ranges[k] > closer_dist){
                            ranges[k] = closer_dist;
                        }
                    }
                }
            }
        }
    }
                   
    float compute_speed(float steering, float front_dist)
    {
        // 1) 조향각 기반 감속
        double angle_factor = std::exp(-k_angle * std::abs(steering));

        // 2) 전방 거리 기반 감속
        double dist_factor = std::clamp(front_dist / front_ref, 0.2, 1.0);

        double speed = this->max_speed * angle_factor * dist_factor;
        return std::max(speed, this->min_speed);
    }
    float avg_range(const std::vector<float>& ranges, int st, int ed)
    {
        float sum = 0.0;
        int cnt = 0;
        for (int i = st; i <= ed; i++) {
            if (ranges[i] > 0.1) {
                sum += ranges[i];
                cnt++;
            }
        }
        return (cnt > 0) ? sum / cnt : 0.0;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   // 1. Replace everything that exceeds the threshold with threshold
        // 2. disparity detection. 
        // 3. disparity_idx~ disparity_idx+carWidth/2 => replace them with min dis
        // 4. gap search 
        // 5. pick Max gap 
        // 6. gap's center idx or farthest point in gap 
        // 7. angle=angle of selected idx 
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        
        /// TODO:
        
        
        this->angle_min=scan_msg->angle_min;
        this->angle_increment=scan_msg->angle_increment;

        std::vector<float> processed_ranges=scan_msg->ranges;
        // 정면 위험 체크
        int center = 540;
        int window = 15;
        float min_front = 100.0;

        for(int i = center - window; i <= center + window; i++){
            min_front = min(min_front, processed_ranges[i]);
        }

        // emergency override
        if(min_front < 0.8){
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = (prev_angle >= 0 ? 0.4 : -0.4);
            drive_msg.drive.speed = 2.0;
            drive_pub_->publish(drive_msg);
            return;
        }
        if(min_front > 4.0)
            threshold = 6.0;
        else if(min_front > 2.0)
            threshold = 4.0;
        else
            threshold = 2.5;

        // 1
        preprocess_lidar(processed_ranges.data());
        // 2+3
        extend_disparities(processed_ranges.data());
        
        // Find max length gap 4+5
        int gap_idx[2];
        find_max_gap(processed_ranges.data(), gap_idx);
        int center_idx = 540;
        float center_dist = processed_ranges[center_idx];
        int best_idx[2]={gap_idx[0],gap_idx[1]};
        if (center_dist < emergency_dist) {
            // 좌 / 우 구간 정의
            float left_avg  = avg_range(processed_ranges, 300, 500);
            float right_avg = avg_range(processed_ranges, 580, 780);

            if (left_avg > right_avg) {
                best_idx[0] = 400;  // 왼쪽으로 강제
            } else {
                best_idx[0] = 680;  // 오른쪽으로 강제
            }
        }
        // Find the best point in the gap 6
        
        find_best_point(processed_ranges.data(), best_idx);
        // 7
        // calculate steering angle from best point's idx
        float new_angle=this->prev_angle*0.7+ 0.3*(this->angle_min+best_idx[0]*this->angle_increment);
        
        // Publish Drive message
        auto drive_msg=ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle=new_angle;
        float front_dist = processed_ranges[540];  // 정중앙
        drive_msg.drive.speed = compute_speed(new_angle, front_dist);
        drive_pub_->publish(drive_msg);

        auto laser_msg = sensor_msgs::msg::LaserScan(*scan_msg); //copy
        laser_msg.ranges=processed_ranges;
        laser_pub_->publish(laser_msg);
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "ego_racecar/laser_model"; // 시뮬레이터 프레임 이름 확인
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 극좌표(거리, 각도)를 직교좌표(x, y)로 변환
        float best_angle = new_angle;
        float best_dist = processed_ranges[best_idx[0]];

        marker.pose.position.x = best_dist * std::cos(best_angle);
        marker.pose.position.y = best_dist * std::sin(best_angle);

        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // 빨간색

        marker_pub_->publish(marker);
        this->prev_angle=new_angle;
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
