#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>

class CheckZValuePositive60 : public BT::SyncActionNode
{
public:
    CheckZValuePositive60(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber zValueSubscriber_;
    double currentZValue_;

    void zValueCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

CheckZValuePositive60::CheckZValuePositive60(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), currentZValue_(0.0)
{
    zValueSubscriber_ = nh_.subscribe("imu_topic", 1, &CheckZValuePositive60::zValueCallback, this);
}

BT::PortsList CheckZValuePositive60::providedPorts()
{
    return { BT::InputPort<double>("min") };
}

BT::NodeStatus CheckZValuePositive60::tick()
{
    double min;
    if (!getInput<double>("min", min)) {
        throw BT::RuntimeError("Missing parameter [min] in CheckZValuePositive60");
    }

    if (currentZValue_ >= 60.0 && currentZValue_ <= 70.0) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void CheckZValuePositive60::zValueCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    currentZValue_ = msg->orientation.z;

    if (currentZValue_ >= 60.0 && currentZValue_ <= 70.0) {
        std::cout << "true" << std::endl;
    }
}

class ReduceLinearActuatorAction : public BT::SyncActionNode
{
public:
    ReduceLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher servoPosePublisher_;
    BT::Blackboard::Ptr blackboard_;
    bool reductionPerformed_;
};

ReduceLinearActuatorAction::ReduceLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), reductionPerformed_(false)
{
    servoPosePublisher_ = config.blackboard->template get<ros::Publisher>("servo_pose_publisher");
    blackboard_ = config.blackboard;
}

BT::PortsList ReduceLinearActuatorAction::providedPorts()
{
    return { BT::InputPort<int>("value") };
}

BT::NodeStatus ReduceLinearActuatorAction::tick()
{
    if (reductionPerformed_) {
        
        return BT::NodeStatus::SUCCESS;
    }

    std::cout << "Executing ReduceLinearActuatorAction" << std::endl;

    
    int currentServoPose;
    if (!blackboard_->get("servo_pose", currentServoPose)) {
        throw BT::RuntimeError("Failed to get current servo_pose value");
    }

    
    int reductionValue;
    if (!getInput<int>("value", reductionValue)) {
        throw BT::RuntimeError("Missing parameter [value] in ReduceLinearActuatorAction");
    }

    
    int newServoPose = currentServoPose + reductionValue;

    
    std_msgs::Int32 reducedValue;
    reducedValue.data = newServoPose;
    servoPosePublisher_.publish(reducedValue);

    
    blackboard_->set("servo_pose", newServoPose);

    
    reductionPerformed_ = true;

    return BT::NodeStatus::SUCCESS;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior_tree_node");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckZValuePositive60>("CheckZValuePositive60");
    factory.registerNodeType<ReduceLinearActuatorAction>("ReduceLinearActuatorAction");

    
    ros::Publisher servoPosePublisher = nh.advertise<std_msgs::Int32>("servo_pose", 1);

    
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("servo_pose_publisher", servoPosePublisher);
    blackboard->set("servo_pose", 255);  

    
    std::string xml_text = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                           "<root BTCPP_format=\"3\">"
                           "  <BehaviorTree ID=\"ReactiveBehaviorTree\">"
                           "    <ReactiveSequence>"
                           "      <CheckZValuePositive60 min=\"60\"/>"
                           "      <ReduceLinearActuatorAction value=\"-25\"/>"
                           "    </ReactiveSequence>"
                           "  </BehaviorTree>"
                           "</root>";

    std::cout << "Loaded XML:\n" << xml_text << std::endl;

    
    auto tree = factory.createTreeFromText(xml_text, blackboard);

    BT::NodeStatus status;
    while (ros::ok()) {
        status = tree.rootNode()->executeTick();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}