#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "activity_pkg/action/move_robo.hpp"

using namespace std::placeholders;
using MoveRobo = activity_pkg::action::MoveRobo;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRobo>;

class MoveRoboServerNode : public rclcpp::Node
{
public:
    MoveRoboServerNode() : Node("move_robo"),intial_position_(50)
    {
        move_robo_server_ = rclcpp_action::create_server<MoveRobo>(
            this, "move_robo",
            std::bind(&MoveRoboServerNode::goalCallBack, this, _1, _2),
            std::bind(&MoveRoboServerNode::cancelCallBack, this, _1),
            std::bind(&MoveRoboServerNode::handleGoalCallBack, this, _1),
            rcl_action_server_get_default_options(),
            cb_group);
        RCLCPP_INFO(this->get_logger(),"Action Server Started");
    }

private:
    rclcpp_action::Server<MoveRobo>::SharedPtr move_robo_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group;
    int intial_position_;

    rclcpp_action::GoalResponse goalCallBack(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobo::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallBack(const std::shared_ptr<GoalHandle> goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleGoalCallBack(const std::shared_ptr<GoalHandle> goal_handle){
        execMove(goal_handle);
    }

    void execMove(const std::shared_ptr<GoalHandle> goal_handle) {
        int final_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;
        int current_position = intial_position_;
        int distance = final_position - current_position;
        while(final_position > current_position) {
            if(distance < velocity) {
                
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRoboServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}