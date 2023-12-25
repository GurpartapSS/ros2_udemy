#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class numberPublisher : public rclcpp::Node
{
public:
    numberPublisher() : Node("Number_Publisher")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&numberPublisher::publishcount, this));
        RCLCPP_INFO(this->get_logger(), "publisher started");
    }

private:
    void publishcount()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number;
        publisher_->publish(msg);
    }
    const int64_t number = 1;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<numberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}