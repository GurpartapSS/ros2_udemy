#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "activity_pkgs/msg/turtle.hpp"
#include "activity_pkgs/msg/turtle_array.hpp"
#include "activity_pkgs/srv/catch_turtle.hpp"

class spawnTurtleNode : public rclcpp::Node
{
public:
    spawnTurtleNode() : Node("spawn_turtle"), turtleCount_(0)
    {
        aliveTurtlePub_ = this->create_publisher<activity_pkgs::msg::TurtleArray>("alive_turtles", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(4),
                                         std::bind(&spawnTurtleNode::timedcallback, this));
        catchTurtleServ_ = this->create_service<activity_pkgs::srv::CatchTurtle>("catch_turtle",
                                                                                 std::bind(&spawnTurtleNode::catchTurtleCallback, this,
                                                                                           std::placeholders::_1, std::placeholders::_2));
                               RCLCPP_INFO(this->get_logger(), "Starting Spawn Node");
    }

private:
    int turtleCount_;
    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<activity_pkgs::msg::Turtle> turtlesList;
    std::vector<std::thread> catchTurtleThread_;
    rclcpp::Publisher<activity_pkgs::msg::TurtleArray>::SharedPtr aliveTurtlePub_;
    rclcpp::Service<activity_pkgs::srv::CatchTurtle>::SharedPtr catchTurtleServ_;

    void catchTurtleCallback(const activity_pkgs::srv::CatchTurtle::Request::SharedPtr request,
                             const activity_pkgs::srv::CatchTurtle::Response::SharedPtr response)
    {
        catchTurtleThread_.push_back(std::thread(std::bind(&spawnTurtleNode::KillTurtleCallback, this, request->name)));
        response->success = true;
    }

    void KillTurtleCallback(std::string name) {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        while(!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),"waiting for server");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            int j = 0;
            for(auto i : turtlesList) {
                if(i.name == name) {
                    turtlesList.erase(turtlesList.begin()+j);
                    publishAliveTurtles();
                    break;
                }
                j++;
            }
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void publishAliveTurtles()
    {
        auto msg = activity_pkgs::msg::TurtleArray();
        msg.turtles = turtlesList;
        aliveTurtlePub_->publish(msg);
    }

    void timedcallback()
    {
        turtleCount_++;
        float x = static_cast<float>(rand() % 11);
        float y = static_cast<float>(rand() % 11);
        float theta = static_cast<float>(rand() % (int)(2 * M_PI));
        std::string name = "sara_clone" + std::to_string(turtleCount_);
        threads_.push_back(std::thread(std::bind(&spawnTurtleNode::spawnTurtle, this,
                                                 x, y, theta, name)));
    }
    void spawnTurtle(float x, float y, float theta, std::string name)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for server");
        }

        auto request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
        request_->x = x;
        request_->y = y;
        request_->theta = theta;
        request_->name = name;

        auto future = client->async_send_request(request_);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        auto turtle = activity_pkgs::msg::Turtle();
        turtle.x = x;
        turtle.y = y;
        turtle.theta = theta;
        turtle.name = name;
        turtlesList.push_back(turtle);
        publishAliveTurtles();
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