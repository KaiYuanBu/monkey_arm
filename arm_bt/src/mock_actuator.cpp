#include "arm_bt/mock_actuator.hpp"

MockActuator::MockActuator() : Node("mock_actuator")
{
    // Declare parameters
    this->declare_parameter("position", 0.0);
    this->declare_parameter("ticks", 5);

    // Get parameters
    position_ = this->get_parameter("position").as_double();
    ticks_ = this->get_parameter("ticks").as_int();

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_callback_handle_ = param_subscriber_->add_parameter_callback(
        "position", 
        std::bind(&MockActuator::on_parameter_changed, this, std::placeholders::_1), 
        this->get_name());

    const std::string node_name = this->get_name();

    // Create GetPosition service
    srv_ = this->create_service<arm_interfaces::srv::GetPosition>(
        node_name + "/GetPosition",
        std::bind(&MockActuator::get_position_callback, this,
            std::placeholders::_1, std::placeholders::_2));
    
    // Create SetTarget action
    action_ = rclcpp_action::create_server<arm_interfaces::action::SetTarget>(
      this,
      node_name + "/SetTarget",
      std::bind(&MockActuator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MockActuator::handle_cancel, this, std::placeholders::_1),
      std::bind(&MockActuator::handle_accepted, this, std::placeholders::_1));
}

// ----- GetPosition service -----

void MockActuator::get_position_callback(
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response)
{   
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Received GetPosition request...");
    response->position = position_;
}

// ----- Parameter changed -----

void MockActuator::on_parameter_changed(const rclcpp::Parameter & p)
{   
    //Update position
    RCLCPP_INFO(
        this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%f\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_double());
    position_ = p.as_double();
}

// ----- SetTarget Action -----

rclcpp_action::GoalResponse MockActuator::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetTarget::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with target %f", goal->target);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MockActuator::handle_cancel(
    const std::shared_ptr<GoalHandleSetTarget> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MockActuator::handle_accepted(const std::shared_ptr<GoalHandleSetTarget> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MockActuator::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void MockActuator::execute(const std::shared_ptr<GoalHandleSetTarget> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetTarget::Feedback>();
    auto & current_position = feedback->current_position;
    auto result = std::make_shared<SetTarget::Result>();

    const double step = (goal->target - position_) / ticks_;

    for (int i = ticks_; (i > 0) && rclcpp::ok(); --i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // Publish feedback
        position_ += step;
        current_position = position_;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Tick #%d", i);
        RCLCPP_INFO(this->get_logger(), "Current position: %f", position_);

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockActuator>());
    rclcpp::shutdown();
    return 0;
}