#include "rclcpp/rclcpp.hpp"
#include "activity_pkgs/srv/set_led.hpp"
#include "activity_pkgs/msg/led_panel_status.hpp"

class ledPanel : public rclcpp::Node
{
public:
    ledPanel() : Node("led_panel_node")
    {
        this->declare_parameter("led_states", std::vector<int64_t>{0, 0, 0});
        led_states_ = this->get_parameter("led_states").as_integer_array();

        publisher_ = this->create_publisher<activity_pkgs::msg::LedPanelStatus>("led_panel_state", 10);
        LedServer_ = this->create_service<activity_pkgs::srv::SetLed>("set_led",
                                                                      std::bind(&ledPanel::setLedState, this, 
                                                                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Led Panel started! looking for battery status");
    }

private:
    std::vector<int64_t> led_states_;
    rclcpp::Publisher<activity_pkgs::msg::LedPanelStatus>::SharedPtr publisher_;
    rclcpp::Service<activity_pkgs::srv::SetLed>::SharedPtr LedServer_;
    void setLedState(const activity_pkgs::srv::SetLed::Request::SharedPtr request_,
                     const activity_pkgs::srv::SetLed::Response::SharedPtr response_)
    {
        led_states_.at(request_->led_number - 1) = request_->state;
        auto msg = activity_pkgs::msg::LedPanelStatus();
        msg.led = led_states_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Led Status 1: %ld, 2: %ld, 3: %ld",msg.led[0],msg.led[1],msg.led[2]);
        response_->success = true;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ledPanel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}