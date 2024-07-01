#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "arm_interfaces/srv/get_position.hpp"
#include "arm_interfaces/action/set_target.hpp"

class MockActuator : public rclcpp::Node
{
    public:
        using GetPosition = arm_interfaces::srv::GetPosition;
        using SetTarget = arm_interfaces::action::SetTarget;
        using GoalHandleSetTarget = rclcpp_action::ServerGoalHandle<SetTarget>;

        MockActuator();

    private:
        // Parameter variables
        double position_;
        int ticks_;

        // Publishers, subscribers, service servers, action servers
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;
        rclcpp::Service<GetPosition>::SharedPtr srv_;
        rclcpp_action::Server<SetTarget>::SharedPtr action_;

        void get_position_callback(
            const std::shared_ptr<GetPosition::Request> request,
            std::shared_ptr<GetPosition::Response> response);
        
        void on_parameter_changed(const rclcpp::Parameter & p);

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const SetTarget::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleSetTarget> goal_handle);
        
        void handle_accepted(const std::shared_ptr<GoalHandleSetTarget> goal_handle);

        void execute(const std::shared_ptr<GoalHandleSetTarget> goal_handle);
};