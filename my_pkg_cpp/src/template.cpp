#include "rclcpp/rclcpp.hpp"

class myNode : public rclcpp::Node
{
public:
    myNode() : Node("Demo_node")
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}