#include "rclcpp/rclcpp.hpp"
#include "activity_pkgs/srv/set_led.hpp"

class batteryStateMonitor : public rclcpp::Node
{
public:
    batteryStateMonitor() : Node("battery_node"),
                            batteryState_(full)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&batteryStateMonitor::checkBatteryState,this));
        last_timechecked_sec = get_clock()->now();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    enum ebatteryState
    {
        full,
        empty
    };
    int8_t batteryState_;
    std::vector<std::thread> threads_;
    rclcpp::Time last_timechecked_sec;
    rclcpp::Time current_time;
    void checkBatteryState()
    {
        current_time = get_clock()->now();
        if (batteryState_ == full)
        {
            if(current_time.seconds() - last_timechecked_sec.seconds() >= 4.0) {
                RCLCPP_INFO(this->get_logger(),"battery is now empty");
                setled(3 , 1);
                batteryState_ = empty;
                last_timechecked_sec = current_time;
            }
        } else if(batteryState_ == empty) {
            if(current_time.seconds() - last_timechecked_sec.seconds() >= 6.0) {
                RCLCPP_INFO(this->get_logger(),"battery is now full");
                setled(3 , 0);
                batteryState_ = full;
                last_timechecked_sec = current_time;
            }
        }
    }
    
    void setled(int8_t ledNumber, int8_t ledState){
        threads_.push_back(std::thread(std::bind(&batteryStateMonitor::callbackledService, this, ledNumber, ledState)));
    }
    
    void callbackledService(int8_t ledNumber, int8_t ledState) {
        auto client = this->create_client<activity_pkgs::srv::SetLed>("set_led");

        while(!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),"waiting for server");
        }

        auto request = std::make_shared<activity_pkgs::srv::SetLed::Request>();
        request->led_number = ledNumber;
        request->state = ledState;

        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"Setting LED %d to %d", (int)request->led_number, (int)request->state);
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<batteryStateMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}