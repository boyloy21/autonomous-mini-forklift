#include "rclcpp/rclcpp.hpp"
#include "forklift_interfaces/msg/current_height.hpp"
#include "forklift_interfaces/msg/lift_limit_status.hpp"

using std::placeholders::_1;

class LiftLimitNode : public rclcpp::Node
{
    public:
        LiftLimitNode()
        : Node("lift_limit_node")
        {
            min_height_= this->declare_parameter("min_height", 0.0);
            max_height_= this->declare_parameter("max_height", 1.2);
            soft_margin_ = this->declare_parameter("soft_margin", 0.05);

            limit_pub_ = this->create_publisher<forklift_interfaces::msg::LiftLimitStatus>(
                "/lift_limit", 10);
            
            height_sub_ = this->create_subscription<forklift_interfaces::msg::CurrentHeight>(
                "/CurrentHeight", 10, std::bind(&LiftLimitNode::heightCallback, this, _1));
            
            RCLCPP_INFO(
                this->get_logger(), "lift_limit_node ready [%.2f, %.2f]", min_height_, max_height_);
        }
    private:
        void heightCallback(const forklift_interfaces::msg::CurrentHeight::SharedPtr msg)
        {
            const double h = msg->current_height;
            auto status = forklift_interfaces::msg::LiftLimitStatus();
            status.top_limit = h >= max_height_;
            status.bottom_limit = h <= min_height_;
            status.soft_limit_exceeded = (h >= max_height_ - soft_margin_) || (h <= min_height_ + soft_margin_);
            status.message = status.top_limit ? "top limit" : status.bottom_limit ? "bottom limit" : 
                            status.soft_limit_exceeded ? "soft limit margin" : "ok";
            limit_pub_->publish(status);
        };

        rclcpp::Subscription<forklift_interfaces::msg::CurrentHeight>::SharedPtr height_sub_;
        rclcpp::Publisher<forklift_interfaces::msg::LiftLimitStatus>::SharedPtr limit_pub_;
        double min_height_;
        double max_height_;
        double soft_margin_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiftLimitNode>());
    rclcpp::shutdown();
    return  0;
}