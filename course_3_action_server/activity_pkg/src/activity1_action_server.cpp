#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "activity_pkg/action/move_robo.hpp"

using namespace std::placeholders;
using MoveRobo = activity_pkg::action::MoveRobo;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRobo>;

class MoveRoboServerNode : public rclcpp::Node
{
public:
    MoveRoboServerNode() : Node("move_robo"),robot_position_(50)
    {
        cb_group_=this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        move_robo_server_ = rclcpp_action::create_server<MoveRobo>(
            this, "move_robo",
            std::bind(&MoveRoboServerNode::goalCallBack, this, _1, _2),
            std::bind(&MoveRoboServerNode::cancelCallBack, this, _1),
            std::bind(&MoveRoboServerNode::handleGoalCallBack, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_);
        RCLCPP_INFO(this->get_logger(),"Action Server Started");
    }

private:
    std::mutex mutex_;
    int robot_position_;
    rclcpp_action::Server<MoveRobo>::SharedPtr move_robo_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<GoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp_action::GoalResponse goalCallBack(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobo::Goal> goal) {
            (void)uuid;
        if( goal->velocity < 0) {
            RCLCPP_INFO(this->get_logger(),"Goal Rejected");
            return rclcpp_action::GoalResponse::REJECT;
        }

        //Policy: preempt previous goal for new one

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_handle_) {
                if(goal_handle_->is_active()) {
                    RCLCPP_INFO(this->get_logger(),"Kat Gaya! Setting new GOAL");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),"Goal Accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallBack(const std::shared_ptr<GoalHandle> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"Goal Canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleGoalCallBack(const std::shared_ptr<GoalHandle> goal_handle){
        execMove(goal_handle);
    }

    void execMove(const std::shared_ptr<GoalHandle> goal_handle) {

        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_handle_ = goal_handle;
        }

        // Get Goal
        int final_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;
        // Execute
        auto result = std::make_shared<MoveRobo::Result>();
        auto feedback = std::make_shared<MoveRobo::Feedback>();

        int distance = final_position - robot_position_;
        while(distance != 0) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position=robot_position_;
                    goal_handle->abort(result);
                    return;
                }
            }
            // Important else the counter will keep going until the goal is reached 
            if(goal_handle->is_canceling()) {
                result->position = robot_position_;
                goal_handle->canceled(result);
                return;
            }
            distance = final_position - robot_position_;
            if(abs(distance) < velocity) {
                RCLCPP_INFO(this->get_logger(),"last step distance: %d position: %d", distance, robot_position_);
                velocity = abs(distance);
            }
            if(distance > 0) {
                robot_position_ += velocity;
            } else if(distance < 0) {
                robot_position_ -= velocity;
            }
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        result->position = robot_position_;
        goal_handle->succeed(result);
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