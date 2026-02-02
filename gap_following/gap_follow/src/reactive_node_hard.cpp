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
        
        this->declare_parameter("emergency_dist", 0.4);
        this->declare_parameter("safe_dist", 1.0);
        this->declare_parameter("disparityT", 0.3);

        this->declare_parameter("k_angle", 2.0);
        this->declare_parameter("front_ref", 2.0);
        this->declare_parameter("max_speed", 6.0);
        this->declare_parameter("min_speed", 1.5);

        this->declare_parameter("threshold_max", 6.0);
        this->declare_parameter("threshold_mid", 4.0);
        this->declare_parameter("threshold_min", 2.5);

        this->declare_parameter("front_max", 4.0);
        this->declare_parameter("front_mid", 2.0);

        emergency_dist = this->get_parameter("emergency_dist").as_double();
        safe_dist = this->get_parameter("safe_dist").as_double();
        disparityT = this->get_parameter("disparityT").as_double();

        k_angle = this->get_parameter("k_angle").as_double();
        front_ref = this->get_parameter("front_ref").as_double();
        min_speed = this->get_parameter("min_speed").as_double();
        max_speed = this->get_parameter("max_speed").as_double();        
        
        threshold_max = this->get_parameter("threshold_max").as_double();
        threshold_mid = this->get_parameter("threshold_mid").as_double();
        threshold_min = this->get_parameter("threshold_min").as_double();

        front_max = this->get_parameter("front_max").as_double();
        front_mid = this->get_parameter("front_mid").as_double();
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
    double rb=0.2; // car width:0.22 -> half_width + margin
    double prev_angle=0.0; // for smoothing

    // scan data
    double angle_min=0.0;
    double angle_increment=0.0;

    double emergency_dist;
    double min_speed;
    double max_speed;
    double front_ref;
    double disparityT ;
    double k_angle;
    double safe_dist ;
    double threshold_max ;
    double threshold_mid ;
    double threshold_min ;
    
    double threshold=threshold_mid;
    double front_max;
    double front_mid;

    void preprocess_lidar(float* ranges){   
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

        // 뒤로 확 회전하지 못하도록 전방 경계값 설정 
        int start_idx = 195; 
        int end_idx = 890;

        //RCLCPP_INFO(this->get_logger(), "--- Gap Search Start ---");

        for(int i = start_idx; i < end_idx; i++){
            // gap으로 보려면 일단 거기까지 거리가 내가 주행 가능한 수준 이상은 되어야 함 
            if(ranges[i] >= safe_dist){ 
                if(cur_len == 0) cur_st = i;
                cur_len++;
            }
            else {
                if(cur_len > 0){
                // 후보 gap들 
                // RCLCPP_INFO(this->get_logger(), "Candidate Gap: st=%d, ed=%d, len=%d", cur_st, i-1, cur_len);
                    // max 갱신하면 현재 정보 저장 + cur_len 초기화 
                    if(cur_len > max_len){
                        max_len = cur_len;
                        max_st = cur_st;
                        max_ed = i - 1;
                    }
                    cur_len = 0;
                }
            }
        }
        
        // 마지막 구간 체크. 마지막 값이 safe_dist보다 커서 갱신 안 됐을 경우 대비해서 체크. 
        if(cur_len > 0){
        // RCLCPP_INFO(this->get_logger(), "Candidate Gap: st=%d, ed=%d, len=%d", cur_st, end_idx-1, cur_len);
            if(cur_len > max_len){
                max_len = cur_len;
                max_st = cur_st;
                max_ed = end_idx - 1;
            }
        }

        //RCLCPP_INFO(this->get_logger(), ">>> Final Selected: st=%d, ed=%d, len=%d", max_st, max_ed, max_len);
        // max gap의 st, ed index 반환 
        indice[0] = max_st;
        indice[1] = max_ed;
    }
            
    void find_best_point(float* ranges, int* indice)
    {   
        int st = indice[0];
        int ed = indice[1];
        float max_val = -1.0;
        // max값과 동일한 idx가 많을 수도 있으니 그 중 첫 번째랑 마지막 인덱스 확인
        // 우리는 그 인덱스들 중 중앙 인덱스를 추종할거임. 처음 나온거 고르면 제일 우측을 추종하게 돼서 벽에 박음;
        int first_idx = -1;
        int last_idx = -1;

        // max gap 순회하면서 최댓값 확인 + 처음/마지막 인덱스 기록
        for (int i = st; i <= ed; i++){
            if (ranges[i] > max_val){
                max_val = ranges[i];
                first_idx = last_idx = i;
            }
            else if (ranges[i] == max_val){
                last_idx = i;
            }
        }
        int gap_center=(st+ed)/2;
        int best_idx =static_cast<int>(0.6 * gap_center + 0.4 * ((first_idx + last_idx) / 2));
        
        indice[0] = best_idx;

        // RCLCPP_INFO(this->get_logger(), "MaxDist: %.2f, BestIdx: %d (Center of indices %d~%d)", max_val, best_idx, first_idx, last_idx);
    }

    void extend_disparities(float* ranges){
        for(int i=0; i<1079; i++){
            float diff = abs(ranges[i] - ranges[i+1]);
            if(diff > disparityT){
                // RCLCPP_INFO(this->get_logger(), "Disparity found at idx %d, diff: %.2f", i, diff);
                
                float closer_dist = min(ranges[i], ranges[i+1]);
                // 차 반폭: rd, 장애물 거리 dist, 각도 theta=atan2(rb,d)니까 ray 개수는 theta/angle_increment
                // 근데 장애물이 멀면 더 적게 막고 가까우면 더 많이 막아야 함  
                int car_idx = static_cast<int>(
                    std::ceil( std::atan2(rb, closer_dist) / angle_increment )
                );


                if (ranges[i] < ranges[i+1]) {
                    int ed =std::min(i + car_idx, 1079); // overflow 방지 
                    for(int k = i + 1; k <= ed; k++){ // 가까운 거리로 다 밀기 
                        if(ranges[k] > closer_dist){
                            ranges[k] = closer_dist;
                            // RCLCPP_INFO(this->get_logger(), "  -> Extending to idx %d", k);
                        }
                    }
                    i = ed; //밀었으니까 i 업데이트해주기 
                }
                else {
                    int st =std::max(0, i + 1 - car_idx);
                    for(int k = st; k <= i; k++){
                        if(ranges[k] > closer_dist){
                            ranges[k] = closer_dist;
                        }
                    }
                }
            }
        }
    }
                   
    float compute_speed(float steering, float front_dist){
        double angle_factor = std::exp(-k_angle * std::abs(steering));
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
    void pub_drive(const sensor_msgs::msg::LaserScan &laser_msg, const std::vector<float> &processed_ranges,float new_angle, int best_idx)
    {
        auto msg = laser_msg;
        msg.ranges = processed_ranges;
        laser_pub_->publish(msg);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/laser_model";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        float dist = processed_ranges[best_idx];
        marker.pose.position.x = dist * std::cos(new_angle);
        marker.pose.position.y = dist * std::sin(new_angle);

        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;

        marker_pub_->publish(marker);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        this->get_parameter("emergency_dist",emergency_dist);
        this->get_parameter("safe_dist",safe_dist);
        this->get_parameter("disparityT",disparityT);

        this->get_parameter("k_angle",k_angle);
        this->get_parameter("front_ref",front_ref);
        this->get_parameter("min_speed",min_speed);
        this->get_parameter("max_speed",max_speed);        
        
        this->get_parameter("threshold_max",threshold_max);
        this->get_parameter("threshold_mid",threshold_mid);
        this->get_parameter("threshold_min",threshold_min);

        this->get_parameter("front_max",front_max);
        this->get_parameter("front_mid",front_mid);
    
    
        /// TODO:

        angle_min=scan_msg->angle_min;
        angle_increment=scan_msg->angle_increment;

        std::vector<float> processed_ranges=scan_msg->ranges;

        // 정면 위험 체크
        int center = 540;
        int window = 15;
        float min_front = 100.0;
        for(int i = center - window; i <= center + window; i++){
            min_front = min(min_front, processed_ranges[i]);
        }
    
        if(min_front > front_max)
            threshold = threshold_max;
        else if(min_front > front_mid)
            threshold = threshold_mid;
        else
            threshold = threshold_min;
        // 1. Replace everything that exceeds the threshold with threshold
        // 2. disparity detection.
        // 3. disparity_idx~ disparity_idx+carWidth/2 => replace them with min dis
        // 4. gap search
        // 5. pick Max gap
        // 6. gap's center idx or farthest point in gap
        // 7. angle=angle of selected idx
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        // 1
        preprocess_lidar(processed_ranges.data());
        // 2+3
        extend_disparities(processed_ranges.data());
        
        // Find max length gap 4+5
        int gap_idx[2];
        find_max_gap(processed_ranges.data(), gap_idx);

        int best_idx[2] = {gap_idx[0], gap_idx[1]};
        // 중앙이 너무 가까우면 best_idx를 강제 조정하여 좌/우측으로 유도
        if (min_front < emergency_dist)
        {
            // 좌 / 우 구간 정의
            float left_avg  = avg_range(processed_ranges, 300, 500);
            float right_avg = avg_range(processed_ranges, 580, 780);
            best_idx[0] = (left_avg > right_avg) ? 400 : 680;
        }

        // Find the best point in the gap 6
        find_best_point(processed_ranges.data(), best_idx);

        //  calculate steering angle from best point's idx 7 (smoothing)
        float new_angle=this->prev_angle*0.7+ 0.3*(this->angle_min+best_idx[0]*this->angle_increment);

        // Publish Drive message
        auto drive_msg=ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle=new_angle;
        // 중앙 전방 거리 기준으로 속도 결정
        float front_dist = processed_ranges[540];  
        drive_msg.drive.speed = compute_speed(new_angle, front_dist);
        drive_pub_->publish(drive_msg);

        pub_drive(*scan_msg, processed_ranges, new_angle, best_idx[0]);

        prev_angle=new_angle;
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
