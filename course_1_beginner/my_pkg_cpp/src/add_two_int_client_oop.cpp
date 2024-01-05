#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class addTwoIntsCleintOOP : public rclcpp::Node
{
public:
    addTwoIntsCleintOOP() : Node("add_Two_Ints_Cleint_OOP")
    {
        thread_ = std::thread(std::bind(&addTwoIntsCleintOOP::addTwoIntsClientNodes,this,
        10, 6));
    }

    void addTwoIntsClientNodes(int a, int b) {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        while(!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),"waiting for server");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"%d + %d = %d",
            (int)request->a, (int)request->b, (int)response->sum);
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    std::thread thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<addTwoIntsCleintOOP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}