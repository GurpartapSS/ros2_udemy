#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class addTwoIntsServerNode : public rclcpp::Node
{
public:
    addTwoIntsServerNode() : Node("add_Two_Ints_Server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                  std::bind(&addTwoIntsServerNode::callbackAddTwoInts, 
                                                                  this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(),"Service Server Started");
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request_,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response_)
    {
        response_->sum = request_->a + request_->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request_->a, (int)request_->b, (int)response_->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<addTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}