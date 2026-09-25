#include <algorithm>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "forklift_interfaces/msg/lift_motor_command.hpp"
#include "forklift_interfaces/msg/lift_encoder_height.hpp"
#include "forklift_interfaces/msg/lift_driver_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Bridges /lift_motor_cmd to the STM32. In sim mode (use_hardware=false),
// default) integrates a simple kinematic model instead of usign serial/CAN.

class LiftDriverNode : public rclcpp::Node
{
    public:
        LiftDriverNode()
        : Node("lift_driver_node")
        {
            use_hardware_ = this->declare_parameter("use_hardware", false);
            min_height_ = this->declare_parameter("min_height", 0.0);
            max_height_ = this->declare_parameter("max_height", 1.2);
            const double rate = this->declare_parameter("publish_rate_hz", 50.0);

            if (use_hardware_) {
                // TODO: open serial/CAN link to STM32 here (libserial /SocketCAN).
                RCLCPP_WARN(
                    this->get_logger(), "use_hardware=true but no backend implementd; usig simulation"
                );
            }

            cmd_sub_ = this->create_subscription<forklift_interfaces::msg::LiftMotorCommand>(
                "/lift_motor_cmd", 10, std::bind(&LiftDriverNode::cmdCallback, this, _1));
            enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/lift_enable", rclcpp::QoS(10).best_effort(), std::bind(&LiftDriverNode::enableCallback, this, _1));
            
            encoder_pub_ = this->create_publisher<forklift_interfaces::msg::LiftEncoderHeight>(
                "/encoder_height", 10);
            status_pub_ = this->create_publisher<forklift_interfaces::msg::LiftDriverStatus>(
                "/lift_drive_status", 10);

            last_tick_ = this->now();
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / rate)), std::bind(&LiftDriverNode::tick, this));
            
            RCLCPP_INFO(
                this->get_logger(), "lift_driver_node ready (%s)",
                use_hardware_ ? "hardware" : "simulation"
            );
        }
        
        private:
            void cmdCallback(const forklift_interfaces::msg::LiftMotorCommand::SharedPtr msg)
            {
                velocity_ = msg->velocity;
                cmd_enable_ = msg->enable;
                // TODO: write (velocity_, cmd_enable_) out to the STM32 in hardware mode.
            }

            void enableCallback(const std_msgs::msg::Bool::SharedPtr msg){
                safety_enable_ = msg->data;
                if (!safety_enable_) {velocity_ = 0.0f;}
            }

            void tick(){
                const rclcpp::Time t = this->now();
                const double dt = (t - last_tick_).seconds();
                last_tick_ = t;

                if (!use_hardware_) {
                    const float v = (safety_enable_ && cmd_enable_) ? velocity_ : 0.0f;
                    height_ = std::clamp(
                        height_ + static_cast<float>(v*dt),
                        static_cast<float>(min_height_), static_cast<float>(max_height_)
                    );

                    auto enc = forklift_interfaces::msg::LiftEncoderHeight();
                    enc.height = height_;
                    enc.raw_ticks = static_cast<int64_t>(height_* 10000.0f);
                    enc.stamp = t;
                    encoder_pub_->publish(enc);
                }
                // TODO : in Hardware mode, read the real encoder value and publish it instead.

                auto status = forklift_interfaces::msg::LiftDriverStatus();
                status.connected = true;
                status.fault = false;
                status.message = use_hardware_ ? "hardware" : "simulation";
                status_pub_->publish(status);
            }

            rclcpp::Subscription<forklift_interfaces::msg::LiftMotorCommand>::SharedPtr cmd_sub_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::LiftEncoderHeight>::SharedPtr encoder_pub_;
            rclcpp::Publisher<forklift_interfaces::msg::LiftDriverStatus>::SharedPtr status_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            float velocity_{0.0f};
            float height_{0.0f};
            bool cmd_enable_{true};
            bool safety_enable_{true};
            rclcpp::Time last_tick_;
            bool use_hardware_;
            double min_height_;
            double max_height_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiftDriverNode>());
    rclcpp::shutdown();
    return 0;
}