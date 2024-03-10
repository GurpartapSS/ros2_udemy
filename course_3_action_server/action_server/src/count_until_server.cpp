#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "udemy_actionserver/action/count_until.hpp"

using CountUntil = udemy_actionserver::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("count_until_server")
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goalCallback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancelCallBack, this, _1),
            std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(), cb_group_);
            RCLCPP_INFO(this->get_logger(),"Action Server Started");
    }

private:

    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<CountUntilGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(),"Received a goal");

            // Policy: refuse new goal is a valid active
            // {
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     if(goal_handle_) {
            //         if(goal_handle_->is_active()) {
            //             RCLCPP_INFO(this->get_logger(),"Goal active, rejecting new one");
            //             return rclcpp_action::GoalResponse::REJECT;
            //         }
            //     }
            // }

            if(goal->target_number <= 0){
                RCLCPP_INFO(this->get_logger(),"Goal Rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }

            //Policy: preempt existing goal for a new valid one
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

    rclcpp_action::CancelResponse cancelCallBack(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            (void) goal_handle;
            RCLCPP_INFO(this->get_logger(),"Request accepted");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        execute_goal(goal_handle);
    }
    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        // Get goal
        int target_number = goal_handle->get_goal()->target_number;
        double period = goal_handle->get_goal()->period;

        // Execute
        auto feedback = std::make_shared<CountUntil::Feedback>();
        int counter = 0;
        rclcpp::Rate loop_rate(1.0/period);
        auto result = std::make_shared<CountUntil::Result>();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for(int i = 0; i < target_number ; i++) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_handle->get_goal_id() == preempted_goal_id_) {
                    RCLCPP_INFO(this->get_logger(),"found the ID stoping this goal");
                    result->reached_number = counter;
                    goal_handle->abort(result);
                    return;
                }
            }
            if(goal_handle->is_canceling()) {
                result->reached_number = counter;
                goal_handle->canceled(result);
                return;
            }
            counter++;
            feedback->current_number = counter;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(),"counter %d", counter);
            loop_rate.sleep();
        }

        // Return response
        result->reached_number = counter;
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}