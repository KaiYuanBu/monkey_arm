#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

using namespace BT;

// Define IsHome action node
class IsHome : public BT::SyncActionNode 
{
  public:
    IsHome(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // Define the tick() function for the IsHome node
    NodeStatus tick() override
    {
        std::cout << "[ IsHome ] => \t" << this->name() << std::endl; 
        return BT::NodeStatus::FAILURE;
    }
};

class SetHomeAction : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    SetHomeAction(const std::string& name, const BT::NodeConfig& config)
      : StatefulActionNode(name, config)
    {}

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<int>("ticks")};
    }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override
    {  
        if ( !getInput<int>("ticks", ticks))
        {
            throw BT::RuntimeError("missing required input [ticks]");
        }
        std::cout << "[ " << this->name() << " ] -> Started with ticks #" << ticks << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override
    {
        std::cout << "[ " << this->name() << " ] -> Tick" << std::endl;
        ticks--;
        if (ticks <= 0) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    // callback to execute if the action was aborted by another node
    void onHalted() override
    {
        std::cout << "[ " << this->name() << " ] -> Aborted" << std::endl;
    }

  private:
    int ticks;
};

// Define the behavior tree structure in XML format
static const char* xml_text = R"(
 <root BTCPP_format="4" main_tree_to_execute = "Home">
    <BehaviorTree ID="Home">
        <Fallback>
            <IsHome name="not at home"/>
            <Parallel>
                <SetHomeAction name="Boom 0" ticks="3"/>
                <SetHomeAction name="Boom 1" ticks="5"/>
                <SetHomeAction name="Boom 2" ticks="5"/>
            </Parallel>
            <IsHome name="at home"/>
        </Fallback>
    </BehaviorTree>
 </root>
 )";

int main()
{   
    BehaviorTreeFactory factory;

    // Register custom action nodes with the factory
    factory.registerNodeType<IsHome>("IsHome");
    factory.registerNodeType<SetHomeAction>("SetHomeAction");

    // Create the behavior tree using the XML description
    auto tree = factory.createTreeFromText(xml_text);

    // Run the behavior tree until it finishes
    tree.tickWhileRunning();

    return 0;
}