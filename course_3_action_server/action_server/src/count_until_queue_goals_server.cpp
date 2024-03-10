#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "udemy_actionserver/action/count_until.hpp"
#include "queue"

using CountUntil = udemy_actionserver::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("count_until_server")
    {
        goal_execute_thread_ = std::thread(&CountUntilServerNode::run_goal_queue,this);
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
    ~CountUntilServerNode() {
        goal_execute_thread_.join();
    }

private:

    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::queue<std::shared_ptr<CountUntilGoalHandle>> goal_queue;
    std::thread goal_execute_thread_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(),"Received a goal");

            if(goal->target_number <= 0){
                RCLCPP_INFO(this->get_logger(),"Goal Rejected");
                return rclcpp_action::GoalResponse::REJECT;
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
    void run_goal_queue() {
        rclcpp::Rate loop_rate(1000.0);
        while(rclcpp::ok()) {
            std::shared_ptr<CountUntilGoalHandle> next_goal;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_queue.size() > 0) {
                    next_goal = goal_queue.front();
                    goal_queue.pop();
                    execute_goal(next_goal);
                }
            }

            loop_rate.sleep();
        }
    }

    void handle_accepted_callback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_queue.push(goal_handle);
            RCLCPP_INFO(this->get_logger(),"Added Goal to queue. Queue Size: %ld",goal_queue.size());
        }
    }
    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
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