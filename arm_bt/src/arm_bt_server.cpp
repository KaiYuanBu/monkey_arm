#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "arm_interfaces/srv/get_position.hpp"
#include "arm_interfaces/srv/set_end_effector_target.hpp"
#include "arm_interfaces/srv/inverse_kinematics.hpp"
#include "arm_interfaces/srv/home_request.hpp"
#include "arm_interfaces/action/set_target.hpp"

#include <unordered_map>
#include <memory>

using namespace BT;



// ====           EDITED (26062024)            ==== //

class HomeRequestService: public RosServiceNode<arm_interfaces::srv::HomeRequest>
{
  public:
    HomeRequestService(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<arm_interfaces::srv::HomeRequest>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      (void)request;
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      bool home_status = response->homing_status;
      std::stringstream ss;

      if (home_status == true)
      {
        // Log
        ss << this->name() << " -> Homing Status: SUCCESS";
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        // setOutput("status", home_status_success_msg);

        return NodeStatus::SUCCESS;
      }

      else
      {
        // Log
        ss << this->name() << " -> Homing Status: FAILURE";
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        // setOutput("status", home_status_failure_msg);

        return NodeStatus::FAILURE;
      }
      
      //return NodeStatus::FAILURE; <-- If error here just add this
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::stringstream ss;
      ss << this->name() << " -> Error: " << error;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      return NodeStatus::FAILURE;
    }
};

// ====           END EDITED            ==== //


class GetPositionService: public RosServiceNode<arm_interfaces::srv::GetPosition>
{
  public:
    GetPositionService(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<arm_interfaces::srv::GetPosition>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name"),
        OutputPort<double>("position")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      (void)request;
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      // Log
      std::stringstream ss;
      ss << this->name() << " -> Position: " << response->position;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      setOutput("position", response->position);
      return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::stringstream ss;
      ss << this->name() << " -> Error: " << error;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      return NodeStatus::FAILURE;
    }
};

class SetTargetAction: public RosActionNode<arm_interfaces::action::SetTarget>
{
public:
  SetTargetAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<arm_interfaces::action::SetTarget>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("action_name"),
      InputPort<double>("target"),
      OutputPort<double>("position")
    });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    // get "target" from the Input port
    getInput("target", goal.target);
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  { 
    (void)wr;

    std::stringstream ss;
    ss << this->name() << " -> succeeded";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    std::stringstream ss;
    ss << this->name() << " -> Error: " << error;
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    // Update position in blackboard
    setOutput("position", feedback->current_position);

    // Logging
    std::stringstream ss;
    ss << this->name() << " -> Current position: " << feedback->current_position;
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    return NodeStatus::RUNNING;
  }
};

class InverseKinematicsService: public RosServiceNode<arm_interfaces::srv::InverseKinematics>
{
  public:
    InverseKinematicsService(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<arm_interfaces::srv::InverseKinematics>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name"),
        InputPort<double>("target_x"),
        InputPort<double>("target_y"),
        InputPort<double>("target_z"),
        OutputPort<double>("b0_target"),
        OutputPort<double>("b1_target"),
        OutputPort<double>("b2_target")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      getInput<double>("target_x", request->target.x);
      getInput<double>("target_y", request->target.y);
      getInput<double>("target_z", request->target.z);

      // Logging
      std::stringstream ss;
      ss << this->name() << " -> x: " << request->target.x 
        << ", y: " << request->target.y
        << ", z: " << request->target.z;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      // Log
      std::stringstream ss;
      ss << this->name() << " -> b0: " << response->b0_target*180/M_PI
        << ", b1: " << response->b1_target
        << ", b2: " << response->b2_target;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      setOutput("b0_target", response->b0_target*180/M_PI);
      setOutput("b1_target", response->b1_target);
      setOutput("b2_target", response->b2_target);
      return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::stringstream ss;
      ss << this->name() << " -> Error: " << error;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      return NodeStatus::FAILURE;
    }
};

class ArmBtServer : public rclcpp::Node
{
  public:
    ArmBtServer(const std::string &node_name)
    : Node(node_name)
    {
      // const std::string node_name = this->get_name();

      //Create set home service
      home_srv_ = this->create_service<std_srvs::srv::Trigger>(
        node_name + "/SetHome", 
        std::bind(&ArmBtServer::set_home, this, std::placeholders::_1, std::placeholders::_2));
      
      //Create set target service
      target_srv_ = this->create_service<arm_interfaces::srv::SetEndEffectorTarget>(
        node_name + "/SetTarget", 
        std::bind(&ArmBtServer::set_target, this, std::placeholders::_1, std::placeholders::_2));
      
      RCLCPP_INFO(this->get_logger(), "%s/%s is ready.", this->get_namespace(), this->get_name());
    }

    enum BtType {HOME, TARGET};

    // Set BT in hashmap
    void set_tree(BtType type, Tree *tree)
    {
      bt_map_[type] = tree;
    }
  
  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
    rclcpp::Service<arm_interfaces::srv::SetEndEffectorTarget>::SharedPtr target_srv_;
    std::unordered_map < BtType, Tree* > bt_map_;

    //Callback for SetHome service
    void set_home(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      RCLCPP_INFO(this->get_logger(), "Received set home request, homing...");

      // Run the behavior tree until it finishes
      NodeStatus status = bt_map_[BtType::HOME]->tickWhileRunning();

      // Set response
      switch (status) {
        case NodeStatus::SUCCESS:
          response->success = true;
          response->message = "Arm has reached home";
          break;
        case NodeStatus::FAILURE:
          response->success = false;
          response->message = "Failed to reach home";
          break;
        case NodeStatus::SKIPPED:
          response->success = false;
          response->message = "Set home BT is skipped";
          break;
        default:
          response->success = false;
          response->message = "Set home unknown error";
          break;
      }
    }

    //Callback for SetEndEffectorTarget service
    void set_target(
      const std::shared_ptr<arm_interfaces::srv::SetEndEffectorTarget::Request> request,
      std::shared_ptr<arm_interfaces::srv::SetEndEffectorTarget::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "Received set set end effector target request, executing...");

      Tree *tree = bt_map_[BtType::TARGET];
      tree->rootBlackboard()->set<double>("target_x", request->target.x);
      tree->rootBlackboard()->set<double>("target_y", request->target.y);
      tree->rootBlackboard()->set<double>("target_z", request->target.z);

      // Run the behavior tree until it finishes
      NodeStatus status = tree->tickWhileRunning();

      // Set response
      switch (status) {
        case NodeStatus::SUCCESS:
          response->success = true;
          response->message = "Target reached";
          break;
        case NodeStatus::FAILURE:
          response->success = false;
          response->message = "Failed to reach target";
          break;
        case NodeStatus::SKIPPED:
          response->success = false;
          response->message = "Set target BT is skipped";
          break;
        default:
          response->success = false;
          response->message = "Set target unknown error";
          break;
      }
    }
};

const char* home_xml = R"(
<root BTCPP_format="4" main_tree_to_execute = "Home">
    <BehaviorTree ID="Home">
        <Sequence>
            <!-- Get current position of all actuators -->
            <Parallel>
                <GetPositionService name="B0" service_name="/arm/b0/GetPosition" position="{b0_position}"/>
                <GetPositionService name="B1" service_name="/arm/b1/GetPosition" position="{b1_position}"/>
                <GetPositionService name="B2" service_name="/arm/b2/GetPosition" position="{b2_position}"/>
            </Parallel>
            <Parallel>
                <!-- Boom 1 not at home? -->
                <Precondition if="b1_position>0.01" else="SUCCESS">
                    <!-- Home Boom 1 -->
                    <HomeRequestService name="B1" service_name="/arm/b1/HomeRequest" />
                </Precondition>
                <Sequence>
                    <!-- Boom 2 not at home? -->
                    <Precondition if="b2_position>0.01" else="SUCCESS">
                        <Sequence>
                            <!-- Bearing not at home? -->
                            <Precondition if="b0_position>0.1 || b0_position<0" else="SUCCESS">
                                <!-- Home Bearing -->
                                <SetTargetAction name="B0" action_name="/arm/b0/SetTarget" target="0"/>
                            </Precondition>
                            <!-- Home Boom 2 -->
                            <HomeRequestService name="B2" service_name="/arm/b2/HomeRequest" />
                        </Sequence>
                    </Precondition>
                    <!-- Bearing not at home? -->
                    <Precondition if="b0_position>0.1 || b0_position<0" else="SUCCESS">
                        <Sequence>
                            <!-- Boom 2 blocking (< 0.05m)? -->
                            <Precondition if="b2_position<0.05" else="FAILURE">
                                <!-- Raise Boom 2 to 0.05m -->
                                <SetTargetAction name="B2" action_name="/arm/b2/SetTarget" target="0.05"/>
                            </Precondition>
                            <!-- Home Bearing -->
                            <SetTargetAction name="B0" action_name="/arm/b0/SetTarget" target="0"/>
                            <!-- Home Boom 2 -->
                            <HomeRequestService name="B2" service_name="/arm/b2/HomeRequest" />
                        </Sequence>
                    </Precondition>
                </Sequence>
            </Parallel>
        </Sequence>
    </BehaviorTree>
</root>
)";

const char* target_xml = R"(
<root BTCPP_format="4" main_tree_to_execute = "Target">
    <BehaviorTree ID="Target">
        <Sequence>
            <!-- Get current position of all actuators -->
            <Parallel>
                <GetPositionService name="B0" service_name="/arm/b0/GetPosition" position="{b0_position}"/>
                <GetPositionService name="B1" service_name="/arm/b1/GetPosition" position="{b1_position}"/>
                <GetPositionService name="B2" service_name="/arm/b2/GetPosition" position="{b2_position}"/>
            </Parallel>
            <!-- Calculate inverse kinematics -->
            <InverseKinematicsService service_name="/arm/calculate"
                target_x="{target_x}" target_y="{target_y}" target_z="{target_z}"
                b0_target="{b0_target}" b1_target="{b1_target}" b2_target="{b2_target}"/>
            <Parallel>
                <!-- Set Boom 2 to target -->
                <SetTargetAction name="B2" action_name="/arm/b2/SetTarget" target="{b2_target}"/>
                <Fallback>
                    <!-- Boom 1 blocking (< 0.05m)? -->
                    <Precondition if="b1_position<0.05" else="FAILURE">
                        <Sequence>
                            <!-- Set Boom 1 to 0.05m -->
                            <SetTargetAction name="B1" action_name="/arm/b1/SetTarget" target="0.05"/>
                            <Parallel>
                                <SetTargetAction name="B0" action_name="/arm/b0/SetTarget" target="{b0_target}"/>
                                <SetTargetAction name="B1" action_name="/arm/b1/SetTarget" target="{b1_target}"/>
                            </Parallel>
                        </Sequence>
                    </Precondition>

                    <!-- Boom 1 not blocking (> 0.05m) -->
                    <Parallel>
                        <SetTargetAction name="B0" action_name="/arm/b0/SetTarget" target="{b0_target}"/>
                        <SetTargetAction name="B1" action_name="/arm/b1/SetTarget" target="{b1_target}"/>
                    </Parallel>

                </Fallback>
            </Parallel>
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{ 
  // Create node
  rclcpp::init(argc, argv);
  auto bt_server_node = std::make_shared<ArmBtServer>("arm_bt_server");

  // Create BT factory
  BehaviorTreeFactory factory;
  RosNodeParams bt_server_params; 
  bt_server_params.nh = bt_server_node;

  // Registor tree nodes
  factory.registerNodeType<GetPositionService>("GetPositionService", bt_server_params);
  factory.registerNodeType<HomeRequestService>("HomeRequestService", bt_server_params);
  factory.registerNodeType<SetTargetAction>("SetTargetAction", bt_server_params);
  factory.registerNodeType<InverseKinematicsService>("InverseKinematicsService", bt_server_params);
  
  // Create the behavior tree using the XML description
  Tree home_tree = factory.createTreeFromText(home_xml);
  Tree target_tree = factory.createTreeFromText(target_xml);
  bt_server_node->set_tree(ArmBtServer::BtType::HOME, &home_tree);
  bt_server_node->set_tree(ArmBtServer::BtType::TARGET, &target_tree);

  // Spin
  rclcpp::spin(bt_server_node);
  rclcpp::shutdown();
  return 0;
}