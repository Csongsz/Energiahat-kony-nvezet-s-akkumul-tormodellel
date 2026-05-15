#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

using namespace std::chrono_literals;

class EnergyOptimizerNode : public rclcpp::Node {
public:
    EnergyOptimizerNode() : Node("energy_optimizer_node") {
        current_mode_ = 3; 

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 10);
        energy_pub_ = this->create_publisher<std_msgs::msg::Float64>("/battery/consumption", 10);
        
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/sim1/targetpoints", 10, 
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                this->last_targets_ = msg;
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&EnergyOptimizerNode::control_loop, this));
    }

private:
    void control_loop() {
        double target_v = 0.0;
        double a_limit = 0.0;
        bool is_curve = false;

        if (last_targets_ && !last_targets_->poses.empty()) {
            double angle = std::abs(std::atan2(last_targets_->poses[0].position.y, last_targets_->poses[0].position.x));
            if (angle > 0.15) is_curve = true;

            if (current_mode_ == 3) { // 3.
                double max_v = 10.0;
                target_v = is_curve ? (max_v / 2.0) : max_v;
                a_limit = 0.8;
            } 
            else if (current_mode_ == 2) { // 2.
                double max_v = 7.0;
                target_v = is_curve ? (max_v / 2.0) : max_v;
                a_limit = 0.1;
            } 
            else { // 1.
                double eco_max_v = 4.0;
                a_limit = 0.1;

                if (is_curve) {
                    target_v = eco_max_v / 2.0;
                    is_coasting_ = false; 
                } else {
                    
                    if (current_v_ >= eco_max_v - 0.1) {
                        is_coasting_ = true; 
                    } else if (current_v_ < 3.2) {
                        is_coasting_ = false; 
                    }

                    if (is_coasting_) {
                        target_v = 0.0; 
                        a_limit = 0.05; 
                    } else {
                        target_v = eco_max_v;
                    }
                }
            }

        } else {
            target_v = 1.5; 
            a_limit = 0.05;
        }

        double diff = target_v - current_v_;
        double step = std::max(-a_limit, std::min(a_limit, diff));
        current_v_ += step;
        
        double power_w = (1500.0 * (step/0.1) + 200.0) * current_v_ + 100.0;
        total_energy_wh_ += (power_w * 0.1) / 3600.0;
        total_dist_m_ += std::abs(current_v_) * 0.1;
        double avg_cons = (total_dist_m_ > 1.0) ? (total_energy_wh_ / (total_dist_m_ / 1000.0)) : 0.0;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = current_v_;
        cmd_pub_->publish(cmd);

        std_msgs::msg::Float64 energy_msg;
        energy_msg.data = total_energy_wh_;
        energy_pub_->publish(energy_msg);

        RCLCPP_INFO(this->get_logger(), 
                    "V: %.2f | Power: %.1f W | Total: %.4f Wh | Avg: %.1f Wh/km", 
                    current_v_, power_w, total_energy_wh_, avg_cons);
    }

    int current_mode_;
    double current_v_ = 0.0;
    double total_energy_wh_ = 0.0;
    double total_dist_m_ = 0.0;
    bool is_coasting_ = false;
    geometry_msgs::msg::PoseArray::SharedPtr last_targets_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr energy_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnergyOptimizerNode>());
    rclcpp::shutdown();
    return 0;
}