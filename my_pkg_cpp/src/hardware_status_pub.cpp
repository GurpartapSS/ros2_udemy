#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class hardwareStatusPub : public rclcpp::Node
{
public:
    hardwareStatusPub() : Node("hardware_Status_Publisher")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&hardwareStatusPub::cPublishHardwareStatus,this));
        RCLCPP_INFO(this->get_logger(),"Hardware Status publisher started");
    }

private:
void cPublishHardwareStatus(){
    auto msg = my_robot_interfaces::msg::HardwareStatus();
    msg.temp = 10;
    msg.are_motor_ready = true;
    msg.debug_msg = "motors are good!";
    pub_->publish(msg);
}
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hardwareStatusPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}