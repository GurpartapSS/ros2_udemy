#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "activity_pkgs/msg/turtle.hpp"
#include "activity_pkgs/msg/turtle_array.hpp"
#include "activity_pkgs/srv/catch_turtle.hpp"

class controlTurtle : public rclcpp::Node
{
public:
    controlTurtle() : Node("catch_turtle")
    {
        subPose_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
                                                                   std::bind(&controlTurtle::cTurtlePose, this, std::placeholders::_1));
        timerMovement_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&controlTurtle::cControlTurtle, this));
        pubTwist_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subAliveTurtles_ = this->create_subscription<activity_pkgs::msg::TurtleArray>("alive_turtles", 10,
                                                                                      std::bind(&controlTurtle::cGetTurtleToCatch, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "turtle controller Created!");
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subPose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwist_;
    rclcpp::Subscription<activity_pkgs::msg::TurtleArray>::SharedPtr subAliveTurtles_;
    std::vector<std::thread> catchThread_;
    activity_pkgs::msg::Turtle turtle_to_catch;
    rclcpp::TimerBase::SharedPtr timerMovement_;
    turtlesim::msg::Pose::SharedPtr msg_;

    void catchTurtle(std::string name) {
        catchThread_.push_back(std::thread(std::bind(&controlTurtle::cathcTurtleClient,
        this, name)));
    }
    void cathcTurtleClient(std::string name)
    {
        auto client = this->create_client<activity_pkgs::srv::CatchTurtle>("catch_turtle");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for server");
        }

        auto request = std::make_shared<activity_pkgs::srv::CatchTurtle::Request>();
        request->name = name;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "catching %s", request->name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void cGetTurtleToCatch(const activity_pkgs::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
        {
            turtle_to_catch = msg->turtles.at(0);
            RCLCPP_INFO(this->get_logger(), "catching turtle %s at x: %f and y: %f", turtle_to_catch.name.c_str(), turtle_to_catch.x, turtle_to_catch.y);
        }
    }
    void cTurtlePose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        msg_ = pose;
    }
    void cControlTurtle()
    {
        if (!turtle_to_catch.name.empty())
        {
            float dist_x = turtle_to_catch.x - msg_->x;
            float dist_y = turtle_to_catch.y - msg_->y;
            float dist = sqrt((dist_x * dist_x) + (dist_y * dist_y));
            auto msg = geometry_msgs::msg::Twist();

            if (dist > 0.5)
            {
                float goal_theta = atan2(dist_y, dist_x);
                float diff = goal_theta - msg_->theta;
                if (diff > M_PI)
                {
                    diff -= 2 * M_PI;
                }
                else if (diff < -M_PI)
                {
                    diff += 2 * M_PI;
                }

                msg.linear.x = 2 * dist;
                msg.angular.z = 6 * diff;
            }
            else
            {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                catchTurtle(turtle_to_catch.name);
                turtle_to_catch.name.clear();
            }
            pubTwist_->publish(msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controlTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}