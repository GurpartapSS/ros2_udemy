#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class numberCounter : public rclcpp::Node
{
public:
    numberCounter() : Node("number_counter"), count_(0)
    {
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                            std::bind(&numberCounter::countcallback, this, std::placeholders::_1));
        publisher_= this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&numberCounter::publishNumer,this));
        RCLCPP_INFO(this->get_logger(), "Counter Started");
        resetService_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
        std::bind(&numberCounter::callbackResetSetBool,this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Reset service Started");
    }

private:

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr resetService_;
    void callbackResetSetBool(const example_interfaces::srv::SetBool::Request::SharedPtr resetRequest) {
        if(resetRequest->data == true) {
            count_ = 0;
            RCLCPP_INFO(this->get_logger(),"reseting count");
        } else {
            RCLCPP_INFO(this->get_logger(),"got reset false");
        }
    }
    void countcallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        count_ += msg->data;
        numberCounter::publishNumer();
    }
    void publishNumer()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = count_;
        publisher_->publish(msg);

    }
    int64_t count_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<numberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}