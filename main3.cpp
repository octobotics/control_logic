#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>  
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
using namespace std::chrono;

class SleepNode : public BT::StatefulActionNode
{
  public:
    SleepNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
      
      return{ BT::InputPort<int>("msec") };
    }

    BT::NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      if( msec <= 0 ) {
        
        return BT::NodeStatus::SUCCESS;
      }
      else {
        
        deadline_ = system_clock::now() + milliseconds(msec);
        return BT::NodeStatus::RUNNING;
      }
    }

   
    BT::NodeStatus onRunning() override
    {
      if ( system_clock::now() >= deadline_ ) {
        return BT::NodeStatus::SUCCESS;
      }
      else {
        return BT::NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "SleepNode interrupted" << std::endl;
    }

  private:
    system_clock::time_point deadline_;
};

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

    void zValueCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

CheckZValuePositive60::CheckZValuePositive60(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), currentZValue_(0.0)
{
    zValueSubscriber_ = nh_.subscribe("angles", 1, &CheckZValuePositive60::zValueCallback, this);
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

void CheckZValuePositive60::zValueCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    currentZValue_ = msg->z;

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

    
    int newServoPose = 230;

    
    std_msgs::Int32 reducedValue;
    reducedValue.data = newServoPose;
    servoPosePublisher_.publish(reducedValue);

    
    blackboard_->set("servo_pose", newServoPose);

    
    reductionPerformed_ = true;

    return BT::NodeStatus::SUCCESS;
}

class TurnOnMotor : public BT::SyncActionNode
{
public:
    TurnOnMotor(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher motorDirectionPublisher_;
    bool motorTurnedOn_;  
};

TurnOnMotor::TurnOnMotor(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), motorTurnedOn_(false)
{
    motorDirectionPublisher_ = config.blackboard->template get<ros::Publisher>("motor_direction_publisher");
}

BT::PortsList TurnOnMotor::providedPorts()
{
    return {};
}

BT::NodeStatus TurnOnMotor::tick()
{
    if (!motorTurnedOn_) {
        std_msgs::Int8 motorDirection;
        motorDirection.data = 2;
        motorDirectionPublisher_.publish(motorDirection);

        std::cout << "Published Motor Direction: " << motorDirection.data << std::endl;

        motorTurnedOn_ = true;  
        return BT::NodeStatus::SUCCESS;
    }

   
    return BT::NodeStatus::SUCCESS;
}






class CheckWireValue : public BT::SyncActionNode
{
public:
    CheckWireValue(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber wireValueSubscriber_;
    float currentWireValue_;
    double minThreshold_;
    double maxThreshold_;

    void wireValueCallback(const std_msgs::Float32::ConstPtr& msg);
};

CheckWireValue::CheckWireValue(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), currentWireValue_(0.0)
{
    wireValueSubscriber_ = nh_.subscribe("wire_value", 1, &CheckWireValue::wireValueCallback, this);

   
    if (!config.blackboard->get("min_threshold", minThreshold_)) {
        throw BT::RuntimeError("Missing parameter [min_threshold] in CheckWireValue");
    }

    if (!config.blackboard->get("max_threshold", maxThreshold_)) {
        throw BT::RuntimeError("Missing parameter [max_threshold] in CheckWireValue");
    }
}

BT::PortsList CheckWireValue::providedPorts()
{
    return {};
}

BT::NodeStatus CheckWireValue::tick()
{
    
    if (currentWireValue_ >= minThreshold_ && currentWireValue_ <= maxThreshold_) {
        std::cout << "true" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void CheckWireValue::wireValueCallback(const std_msgs::Float32::ConstPtr& msg)
{
    currentWireValue_ = msg->data;
}


class TurnOffMotor : public BT::SyncActionNode
{
public:
    TurnOffMotor(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher motorDirectionPublisher_;
    bool motorTurnedOff_; 
};

TurnOffMotor::TurnOffMotor(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), motorTurnedOff_(false)
{
    motorDirectionPublisher_ = config.blackboard->template get<ros::Publisher>("motor_direction_publisher");
}

BT::PortsList TurnOffMotor::providedPorts()
{
    return {};
}

BT::NodeStatus TurnOffMotor::tick()
{
    if (!motorTurnedOff_) {
        std_msgs::Int8 motorDirection;
        motorDirection.data = 0;  
        motorDirectionPublisher_.publish(motorDirection);

        std::cout << "Published Motor Direction: " << motorDirection.data << std::endl;

        motorTurnedOff_ = true;  
        return BT::NodeStatus::SUCCESS;
    }

    
    return BT::NodeStatus::SUCCESS;
}

class IncreaseLinearActuatorAction : public BT::SyncActionNode
{
public:
    IncreaseLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher servoPosePublisher_;
    BT::Blackboard::Ptr blackboard_;
    bool increasePerformed_;
};

IncreaseLinearActuatorAction::IncreaseLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), increasePerformed_(false)
{
    servoPosePublisher_ = config.blackboard->template get<ros::Publisher>("servo_pose_publisher");
    blackboard_ = config.blackboard;
}

BT::PortsList IncreaseLinearActuatorAction::providedPorts()
{
    return { BT::InputPort<int>("value") };
}

BT::NodeStatus IncreaseLinearActuatorAction::tick()
{
    if (increasePerformed_) {
        return BT::NodeStatus::SUCCESS;
    }

    std::cout << "Executing IncreaseLinearActuatorAction" << std::endl;

    int currentServoPose;
    if (!blackboard_->get("servo_pose", currentServoPose)) {
        throw BT::RuntimeError("Failed to get current servo_pose value");
    }

    int increaseValue;
    if (!getInput<int>("value", increaseValue)) {
        throw BT::RuntimeError("Missing parameter [value] in IncreaseLinearActuatorAction");
    }

    int newServoPose = 255;

    std_msgs::Int32 increasedValue;
    increasedValue.data = newServoPose;
    servoPosePublisher_.publish(increasedValue);

    blackboard_->set("servo_pose", newServoPose);

    increasePerformed_ = true;

    return BT::NodeStatus::SUCCESS;
}

class TurnOnMotorOpposite : public BT::SyncActionNode
{
public:
    TurnOnMotorOpposite(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher motorDirectionPublisher_;
    bool motorTurnedOn_;  
};

TurnOnMotorOpposite::TurnOnMotorOpposite(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), motorTurnedOn_(false)
{
    motorDirectionPublisher_ = config.blackboard->template get<ros::Publisher>("motor_direction_publisher");
}

BT::PortsList TurnOnMotorOpposite::providedPorts()
{
    return {};
}

BT::NodeStatus TurnOnMotorOpposite::tick()
{
    if (!motorTurnedOn_) {
        std_msgs::Int8 motorDirection;
        motorDirection.data =   1;
        motorDirectionPublisher_.publish(motorDirection);

        std::cout << "Published Motor Direction: " << motorDirection.data << std::endl;

        motorTurnedOn_ = true;  
        return BT::NodeStatus::SUCCESS;
    }

    
    return BT::NodeStatus::SUCCESS;
}
ros::Publisher motorDirectionPublisher;  
std_msgs::Float32 wireValueMsg;

void wireValueCallback(const std_msgs::Float32::ConstPtr& msg) {
    wireValueMsg = *msg;

    if (wireValueMsg.data <= 0.012) {
        std_msgs::Int8 motorDirectionMsg;
        motorDirectionMsg.data = 0;
        motorDirectionPublisher.publish(motorDirectionMsg);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_node");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckZValuePositive60>("CheckZValuePositive60");
    factory.registerNodeType<ReduceLinearActuatorAction>("ReduceLinearActuatorAction");
    factory.registerNodeType<TurnOnMotor>("TurnOnMotor");
    factory.registerNodeType<CheckWireValue>("CheckWireValue");
    factory.registerNodeType<TurnOffMotor>("TurnOffMotor");
    factory.registerNodeType<IncreaseLinearActuatorAction>("IncreaseLinearActuatorAction");

    ros::Publisher servoPosePublisher = nh.advertise<std_msgs::Int32>("servo_pose", 1);
    motorDirectionPublisher = nh.advertise<std_msgs::Int8>("motor_direction", 1);
    ros::Publisher wireValuePublisher = nh.advertise<std_msgs::Float32>("wire_value", 1);
    

    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("servo_pose_publisher", servoPosePublisher);
    blackboard->set("servo_pose", 255);
    blackboard->set("motor_direction_publisher", motorDirectionPublisher);

    
    for(int i = 1; i <= 5; i++)
    {std::vector<std::pair<double, double>> wireValueThresholds = {
        {0.024, 0.026}, {0.048, 0.054}, {0.072, 0.081}, {0.096, 0.108},
        {0.12, 0.135}, {0.144, 0.162}, {0.168, 0.189}, {0.192, 0.216}
    };
    for (int iteration = 1; iteration <= 8; ++iteration) {
        double minThreshold, maxThreshold;
        std::tie(minThreshold, maxThreshold) = wireValueThresholds[iteration - 1];

        
        blackboard->set("min_threshold", minThreshold);
        blackboard->set("max_threshold", maxThreshold);

        std::string xml_text = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                            "<root BTCPP_format=\"3\">"
                            "  <BehaviorTree ID=\"ReactiveBehaviorTree\">"
                            "    <ReactiveSequence>"
                            "      <CheckZValuePositive60 min=\"60\"/>"
                            "      <ReduceLinearActuatorAction value=\"-25\"/>"
                            "      <TurnOnMotor/>"
                            "      <CheckWireValue/>"
                            "      <TurnOffMotor/>"
                            "      <IncreaseLinearActuatorAction value=\"25\"/>"
                            "    </ReactiveSequence>"
                            "  </BehaviorTree>"
                            "</root>";

        std::cout << "Loaded XML:\n" << xml_text << std::endl;

        auto tree = factory.createTreeFromText(xml_text, blackboard);

        BT::NodeStatus status;
        do {
            status = tree.rootNode()->executeTick();
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        } while (status != BT::NodeStatus::SUCCESS);
    }

    ros::Duration(5.0).sleep();

   
    ros::Subscriber wireValueSub = nh.subscribe("/wire_value", 1, wireValueCallback);

   
    std_msgs::Int8 motorDirectionMsg;
    motorDirectionMsg.data = 1;
    motorDirectionPublisher.publish(motorDirectionMsg);
    ros::spinOnce(); 
    ros::Duration(1.0).sleep(); 

    
    while (ros::ok()) {
        ros::spinOnce();  
        ros::Duration(1.0).sleep(); 

       
        if (wireValueMsg.data >= 0.012 && wireValueMsg.data <= 0.016) {
            
            motorDirectionMsg.data = 0;
            motorDirectionPublisher.publish(motorDirectionMsg);
            ros::spinOnce();  
            break;  
        }
    }

    ros::Duration(5.0).sleep();
    }
    return 0;
}
