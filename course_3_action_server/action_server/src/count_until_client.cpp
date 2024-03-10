#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include<udemy_actionserver/action/count_until.hpp>

using CountUntil = udemy_actionserver::action::CountUntil;
using GoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUnitlClient : public rclcpp::Node
{
public:
    CountUnitlClient() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this,
        "count_until");
    }

    void send_goal(int target_number, double period) {
        //wati for server
        count_until_client_->wait_for_action_server();

        //add goal
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        //add goal options
        auto goal_options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        goal_options.result_callback = std::bind(&CountUnitlClient::goalCallBack, this, _1);
        goal_options.goal_response_callback = std::bind(&CountUnitlClient::goalResponseCallBack, this, _1);
        goal_options.feedback_callback = std::bind(&CountUnitlClient::feedbackCallBack, this, _1, _2);

        RCLCPP_INFO(this->get_logger(),"Sending a goal");
        count_until_client_->async_send_goal(goal, goal_options);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CountUnitlClient::timercallback, this));
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    GoalHandle::SharedPtr goal_;
    void goalCallBack(const GoalHandle::WrappedResult &result) {

        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),"Goal Succeeded");
        } else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Aborted");
        } else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Canceled");
        }
            int reached_number = result.result->reached_number;
            RCLCPP_INFO(this->get_logger(),"Reached Number: %d",reached_number);
        
    }

    void goalResponseCallBack(const GoalHandle::SharedPtr &goal_handle) {
        if(!goal_handle) {
            RCLCPP_INFO(this->get_logger(),"Goal Rejected :(");
        } else {
            RCLCPP_INFO(this->get_logger(),"Goal Accepted :D");
            goal_ = goal_handle;
        }
    }

    void feedbackCallBack( const GoalHandle::SharedPtr &goal_handle,
    const std::shared_ptr<const CountUntil::Feedback> feedback) {
        (void)goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(),"Feedback %d",number);
    }

    void timercallback() {
        // RCLCPP_INFO(this->get_logger(),"cancel goal");
        // count_until_client_->async_cancel_goal(goal_);
        timer_->cancel();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUnitlClient>();
    node->send_goal(6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}