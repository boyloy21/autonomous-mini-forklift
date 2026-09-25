#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include "forklift_interfaces/msg/lift_encoder_height.hpp"
#include "forklift_interfaces/msg/current_height.hpp"

using std::placeholders::_1;

class LiftFeedbackNode : public rclcpp::Node
{
    public:
        LiftFeedbackNode()
        : Node("lift_feedback_node")
        {
            moving_threshold_ = this->declare_parameter("moving_velocity_threshold", 0.005);
            height_pub_ = this->create_publisher<forklift_interfaces::msg::CurrentHeight>(
                "/CurrentHeight", 10);
            encoder_sub_ = this->create_subscription<forklift_interfaces::msg::LiftEncoderHeight>(
                "/encoder_height", 10, std::bind(&LiftFeedbackNode::encoderCallback, this, _1));

            RCLCPP_INFO(this->get_logger(), "lift_feedback_node ready");
        }

    private:
        void encoderCallback(const forklift_interfaces::msg::LiftEncoderHeight::SharedPtr msg)
        {
            const rclcpp::Time now = this->now();
            float velocity = 0.0f;

            if (last_height_.has_value() && last_time_.has_value()) {
                const double dt = (now - *last_time_).seconds();
                if (dt > 1e-4) {
                    velocity = static_cast<float>((msg->height - *last_height_) / dt);
                }
            }
            last_height_ = msg->height;
            last_time_ = now;

            auto out = forklift_interfaces::msg::CurrentHeight();
            out.current_height = msg->height;
            out.velocity = velocity;
            out.moving = std::fabs(velocity) > moving_threshold_;
            height_pub_->publish(out);
        }

        rclcpp::Subscription<forklift_interfaces::msg::LiftEncoderHeight>::SharedPtr encoder_sub_;
        rclcpp::Publisher<forklift_interfaces::msg::CurrentHeight>::SharedPtr height_pub_;
        std::optional<float> last_height_;
        std::optional<rclcpp::Time> last_time_;
        double moving_threshold_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiftFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}