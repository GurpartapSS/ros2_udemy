#include "rclcpp/rclcpp.hpp"
#include "activity_pkgs/srv/set_led.hpp"

class batteryStatePub : public rclcpp::Node
{
    public:
        batteryStatePub() : Node("battery_state"),
        batteryLevel_(100),
        ledSetStatus_(0),
        read_t_4(true),
        read_t_6(false)
        {
            thread_ = std::thread(std::bind(&batteryStatePub::update,this));
        }

        void setLed(int ledNumber, bool set) {
            auto client = this->create_client<activity_pkgs::srv::SetLed>("set_led");

            while(!client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(),"waiting for server");
            }

            auto request = std::make_shared<activity_pkgs::srv::SetLed::Request>();
            if(set) {
                ledSetStatus_ |= (1 << ledNumber);
            } else {
                ledSetStatus_ &= ~(1 << ledNumber);
            }
            request->leds = ledSetStatus_;
            auto future = client->async_send_request(request);
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(),"setting led %d to %d", ledNumber, set);
            } catch(const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
    }

    private:
        std::thread thread_;
        int8_t batteryLevel_;
        int8_t ledSetStatus_;
        bool read_t_4;
        bool read_t_6;
        rclcpp::TimerBase::SharedPtr timer_;
        void update() {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),std::bind(&batteryStatePub::checkBattery, this));
        }
        void checkBattery() {
            rclcpp::Time current_time = get_clock()->now();
            if( read_t_4 && current_time.seconds() >= 4.0) {
                RCLCPP_INFO(this->get_logger(),"4 seconds passed, battery empty");
                read_t_4 = false;
                batteryStatePub::setLed(1, true);
                read_t_6 = true;
            } else if( read_t_6 && current_time.seconds() >= 6.0) {
                RCLCPP_INFO(this->get_logger(),"4 seconds passed, battery empty");
                read_t_6 = false;
                batteryStatePub::setLed(1, true);
                timer_->cancel();
                timer_->reset();
                RCLCPP_INFO(get_logger(), "Timer reset");
                read_t_4 = true;
            }
        }


};

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<batteryStatePub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}