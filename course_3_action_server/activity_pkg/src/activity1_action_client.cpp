#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "activity_pkg/action/move_robo.hpp"
#include "activity_pkg/msg/cancel_move.hpp"

using MoveRobo = activity_pkg::action::MoveRobo;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveRobo>;
using namespace std::placeholders;
class MoveRoboClientNode : public rclcpp::Node
{
public:
    MoveRoboClientNode() : Node("move_robo_client")
    {
        move_robo_client_ = rclcpp_action::create_client<MoveRobo>(this,
        "move_robo");
    cancel_subscriber_ = this->create_subscription<activity_pkg::msg::CancelMove>("cancel_move", 10,
        std::bind(&MoveRoboClientNode::subCancelCallBack, this, _1));
    }


    void send_goal(int position, int velocity) {
        move_robo_client_->wait_for_action_server();

        auto goal = MoveRobo::Goal();
        goal.position = position;
        goal.velocity = velocity;


        auto goal_options = rclcpp_action::Client<MoveRobo>::SendGoalOptions();
        goal_options.result_callback = std::bind(&MoveRoboClientNode::goalCallBack, this, _1);
        goal_options.feedback_callback = std::bind(&MoveRoboClientNode::feedbackCallBack, this, _1, _2);
        goal_options.goal_response_callback = std::bind(&MoveRoboClientNode::goalResponseCallBack, this, _1);
        RCLCPP_INFO(this->get_logger(), "Sending goal!");
        move_robo_client_->async_send_goal(goal, goal_options);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MoveRoboClientNode::timerCallBack, this));
    }

private:
    rclcpp_action::Client<MoveRobo>::SharedPtr move_robo_client_;
    GoalHandle::SharedPtr goal_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscriber for cancel cmd
    rclcpp::Subscription<activity_pkg::msg::CancelMove>::SharedPtr cancel_subscriber_;

    void subCancelCallBack(activity_pkg::msg::CancelMove::SharedPtr msg) {
        (void)msg;
        // if(msg->data == 1) {
            if(goal_){
                move_robo_client_->async_cancel_goal(goal_);
            }
        // }
    }

    void goalResponseCallBack(std::shared_ptr<GoalHandle> goal_handle) {
        if(!goal_handle) {
            RCLCPP_INFO(this->get_logger()," Goal Rejected :()");
        } else {
            RCLCPP_INFO(this->get_logger()," Goal Accepted :D)");
            goal_ = goal_handle;

        }
    }

    void goalCallBack(GoalHandle::WrappedResult result) {
        auto status  = result.code;
        if(status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),"Goal Succeeded");
        } else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Aborted");
        } else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Canceled");
        }

        int position = result.result->position;
        RCLCPP_INFO(this->get_logger(),"Position %d", position);
    }

    void feedbackCallBack(std::shared_ptr<GoalHandle> goal_handle,
    const std::shared_ptr<const MoveRobo::Feedback> feedback) {
        (void)goal_handle;
        int position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(),"Current Position: %d", position);
    }

    void timerCallBack() {
        // RCLCPP_INFO(this->get_logger(),"cancel goal");
        // move_robo_client_->async_cancel_goal(goal_);
        timer_->cancel();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRoboClientNode>();
    node->send_goal(1000,2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}