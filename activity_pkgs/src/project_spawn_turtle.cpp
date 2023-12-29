#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

class spawnTurtleNode : public rclcpp::Node
{
public:
    spawnTurtleNode() : Node("spawn_turtle"), turtleCount_(0)
    {
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&spawnTurtleNode::timedcallback, this));
        RCLCPP_INFO(this->get_logger(), "Starting Spawn Node");
    }

private:
    int turtleCount_;
    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timedcallback()
    {
        turtleCount_++;
        float y = 1.0;
        if(turtleCount_%2 == 0) {
            y = 5.0;
        }
        std::string name = "sara" + std::to_string(turtleCount_);
        threads_.push_back(std::thread(std::bind(&spawnTurtleNode::spawnTurtle, this,
        4.0, y, 0.0, name)));
    }
    void spawnTurtle(float x, float y, float theta, std::string name)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while(!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),"waiting for server");
        }

        auto request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
        request_->x = x;
        request_->y = y;
        request_->theta = theta;
        request_->name = name;

        auto future = client->async_send_request(request_);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),name.c_str());
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<spawnTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}