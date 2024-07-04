#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/decorators/timer_queue.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <csignal>
#include <unistd.h>  // For getuid() and getpwuid()
#include <pwd.h>     // For getpwuid()


using namespace std::chrono;

std_msgs::Float32 positiveAngle; 
std_msgs::Float32 negativeAngle; 

void positiveAngleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    positiveAngle = *msg;
}

void negativeAngleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    negativeAngle = *msg;
}

class Sleep : public BT::SyncActionNode
{
public:
    Sleep(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    int duration_;
};

Sleep::Sleep(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList Sleep::providedPorts()
{
    return {BT::InputPort<int>("duration")};
}

BT::NodeStatus Sleep::tick()
{
   if(!getInput<int>("duration", duration_))
   {
    throw BT::RuntimeError("Missing parameter [duration] in Sleep");
   }
   std::this_thread::sleep_for(std::chrono::seconds(duration_));
   std::cout << "Sleep for:" << duration_ << "seconds" << std::endl;
   return BT::NodeStatus::SUCCESS;
}


class CheckZValuePositive60 : public BT::ConditionNode
{
public:
    CheckZValuePositive60(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

    void reset();

private:
    ros::NodeHandle nh_;
    ros::Subscriber zValueSubscriber_;
    double currentZValue_;
    BT::NodeStatus status_;

    void zValueCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

CheckZValuePositive60::CheckZValuePositive60(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), currentZValue_(0.0), status_(BT::NodeStatus::FAILURE)
{
    zValueSubscriber_ = nh_.subscribe("/imu/angles", 1, &CheckZValuePositive60::zValueCallback, this);
}

BT::PortsList CheckZValuePositive60::providedPorts()
{
    return {BT::InputPort<double>("min")};
}

void CheckZValuePositive60::reset()
{
    status_ = BT::NodeStatus::FAILURE;
    currentZValue_ = 0.0; 
}

BT::NodeStatus CheckZValuePositive60::tick()
{
    double minPositive = positiveAngle.data;
    double minNegative = negativeAngle.data;

    if ((currentZValue_ >= minPositive && currentZValue_ <= minPositive + 10.0) || 
        (currentZValue_ >= minNegative && currentZValue_ <= minNegative + 10.0)) {
        std::cout << "executed zed" << std::endl;
        status_ = BT::NodeStatus::SUCCESS;
    } else {
        status_ = BT::NodeStatus::FAILURE;
    }

    return status_;
}

void CheckZValuePositive60::zValueCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    currentZValue_ = msg->y;
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
};

ReduceLinearActuatorAction::ReduceLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
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
    std::cout << "Executing ReduceLinearActuatorAction" << std::endl;

    int currentServoPose;
    if (!blackboard_->get("servo_pose", currentServoPose)) {
        throw BT::RuntimeError("Failed to get current servo_pose value");
    }

    int reductionValue;
    if (!getInput<int>("value", reductionValue)) {
        throw BT::RuntimeError("Missing parameter [value] in ReduceLinearActuatorAction");
    }

    int newServoPose = 65;

    std_msgs::Int32 reducedValue;
    reducedValue.data = newServoPose;
    servoPosePublisher_.publish(reducedValue);

    blackboard_->set("servo_pose", newServoPose);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "LAC Reduce Slept for 10 seconds" << std::endl;

    return BT::NodeStatus::SUCCESS;
}

class TurnOnMotor : public BT::SyncActionNode
{
public:
    TurnOnMotor(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::ServiceClient motorServiceClient_;
    bool motorTurnedOn_;
};

TurnOnMotor::TurnOnMotor(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), motorTurnedOn_(false)
{
    motorServiceClient_ = nh_.serviceClient<std_srvs::Trigger>("/clockwise");
}

BT::PortsList TurnOnMotor::providedPorts()
{
    return {};
}

BT::NodeStatus TurnOnMotor::tick()
{
    
        std_srvs::Trigger srv;
        if (motorServiceClient_.call(srv)) {
            if (srv.response.success) {
                std::cout << "Motor turned on successfully." << std::endl;
                motorTurnedOn_ = true;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "Failed to turn on motor: " << srv.response.message << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        } else {
            std::cout << "Service call to /clockwise failed." << std::endl;
            return BT::NodeStatus::FAILURE;
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
        std::cout << "inside wire value" << std::endl;
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
};

IncreaseLinearActuatorAction::IncreaseLinearActuatorAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
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
    std::cout << "Executing IncreaseLinearActuatorAction" << std::endl;

    int currentServoPose;
    if (!blackboard_->get("servo_pose", currentServoPose)) {
        throw BT::RuntimeError("Failed to get current servo_pose value");
    }

    int increaseValue;
    if (!getInput<int>("value", increaseValue)) {
        throw BT::RuntimeError("Missing parameter [value] in IncreaseLinearActuatorAction");
    }

    int newServoPose = 90;

    std_msgs::Int32 increasedValue;
    increasedValue.data = newServoPose;
    servoPosePublisher_.publish(increasedValue);

    blackboard_->set("servo_pose", newServoPose);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "LAC increase slept for 10s" << std::endl;

    return BT::NodeStatus::SUCCESS;
}

class ActiveAuto : public BT::SyncActionNode
{
public:
    ActiveAuto(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher activatePublisher_;
};

ActiveAuto::ActiveAuto(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    activatePublisher_ = config.blackboard->template get<ros::Publisher>("activate");
}

BT::PortsList ActiveAuto::providedPorts()
{
    return {};
}

BT::NodeStatus ActiveAuto::tick()
{
    std_msgs::Int8 activateMsg;
    activateMsg.data = 1;
    activatePublisher_.publish(activateMsg);
    std::cout << "Published Auto Direction: " << static_cast<int>(activateMsg.data) << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ActivateStop Node
class ActivateStop : public BT::SyncActionNode
{
public:
    ActivateStop(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher activatePublisher_;
};

ActivateStop::ActivateStop(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    activatePublisher_ = config.blackboard->template get<ros::Publisher>("navigation_control");
}

BT::PortsList ActivateStop::providedPorts()
{
    return {};
}

BT::NodeStatus ActivateStop::tick()
{
    std_msgs::Int32 activateMsg;
    activateMsg.data = 0;
    activatePublisher_.publish(activateMsg);
    std::cout << "Published Auto Direction: " << static_cast<int>(activateMsg.data) << std::endl;
    return BT::NodeStatus::SUCCESS;
}




class ActivateReverse : public BT::SyncActionNode
{
public:
    ActivateReverse(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::Publisher activatePublisher_;
    BT::Blackboard::Ptr blackboard_;
    CheckZValuePositive60* checkZValueNode_;
    int counter_;
};

ActivateReverse::ActivateReverse(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), counter_(0)
{
    activatePublisher_ = config.blackboard->template get<ros::Publisher>("navigation_control");
    blackboard_ = config.blackboard;

    checkZValueNode_ = new CheckZValuePositive60("CheckZValuePositive60", config);
    blackboard_->set("CheckZValuePositive60", checkZValueNode_);
}

BT::PortsList ActivateReverse::providedPorts()
{
    return {};
}

BT::NodeStatus ActivateReverse::tick()
{
    counter_++; 

    std_msgs::Int32 activateMsg;
    if (counter_ % 2 == 0)
    {
        activateMsg.data = 1;
    }
    else
    {
        activateMsg.data = 2;
    }
    
    activatePublisher_.publish(activateMsg);
    std::cout << "Published Auto Direction: " << static_cast<int>(activateMsg.data) << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    
    checkZValueNode_->reset();
    std::cout << "CheckZValuePositive60 node status reset." << std::endl;

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
        motorDirection.data = 2;
        motorDirectionPublisher_.publish(motorDirection);

        std::cout << "Published Motor Direction: " << motorDirection.data << std::endl;

        motorTurnedOn_ = true;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
}

ros::Publisher motorDirectionPublisher;
ros::Publisher servoPosePublisher1;
ros::Publisher autoModePublisher1;
std_msgs::Float32 wireValueMsg;

void wireValueCallback(const std_msgs::Float32::ConstPtr& msg)
{
    wireValueMsg = *msg;

    if (wireValueMsg.data <= 0.012) {
        std_msgs::Int8 motorDirectionMsg;
        motorDirectionMsg.data = 0;
        motorDirectionPublisher.publish(motorDirectionMsg);
        //ros::spinOnce();
    }
}

void signal_handler(int signal) 
{
    if (signal == SIGABRT)
        std::cerr << "SIGABRT received\n";
    else
        std::cerr << "Unexpected signal " << signal << " received\n";
    std::_Exit(EXIT_FAILURE);
}


bool terminateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    auto previous_handler = std::signal(SIGABRT, signal_handler);
    if (previous_handler == SIG_ERR)
    {
        std::cerr << "Setup failed\n";
        return EXIT_FAILURE;
    }
 
    std::abort(); // Raise SIGABRT
    std::cout << "This code is unreachable\n"; 
    return true;
    
}

std::string getHomeDirectory() {
    struct passwd *pw = getpwuid(getuid());
    return std::string(pw->pw_dir);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior_tree_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckZValuePositive60>("CheckZValuePositive60");
    factory.registerNodeType<ReduceLinearActuatorAction>("ReduceLinearActuatorAction");
    factory.registerNodeType<TurnOnMotor>("TurnOnMotor");
    factory.registerNodeType<CheckWireValue>("CheckWireValue");
    factory.registerNodeType<TurnOffMotor>("TurnOffMotor");
    factory.registerNodeType<IncreaseLinearActuatorAction>("IncreaseLinearActuatorAction");
    factory.registerNodeType<ActiveAuto>("ActiveAuto");
    factory.registerNodeType<ActivateStop>("ActivateStop");
    factory.registerNodeType<ActivateReverse>("ActivateReverse");
    factory.registerNodeType<Sleep>("Sleep");


    positiveAngle.data = 60.0;
    negativeAngle.data = 290.0;
    ros::Subscriber positiveAngleSubscriber = nh.subscribe("positive_angle", 10, positiveAngleCallback);
    ros::Subscriber negativeAngleSubscriber = nh.subscribe("negative_angle", 10, negativeAngleCallback);
    // ros::Publisher positiveAnglePublisher = nh.advertise<std_msgs::Float32>("positive_angle", 1);
    // ros::Publisher negativeAnglePublisher = nh.advertise<std_msgs::Float32>("negative_angle", 1);

    ros::Publisher servoPosePublisher = nh.advertise<std_msgs::Int32>("servo_pose", 1);
    motorDirectionPublisher = nh.advertise<std_msgs::Int8>("motor_direction", 1);
    servoPosePublisher = nh.advertise<std_msgs::Int32>("servo_pose", 1);
    ros::Publisher wireValuePublisher = nh.advertise<std_msgs::Float32>("wire_value", 1);
    ros::Publisher activatePublisher = nh.advertise<std_msgs::Int32>("navigation_control", 1);
    ros::Publisher lateralShiftPublisher = nh.advertise<std_msgs::Int32>("/lateral_shift", 1);
    ros::ServiceServer terminateService = nh.advertiseService("terminate_program", terminateCallback);

    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("servo_pose_publisher", servoPosePublisher);
    blackboard->set("servo_pose", 255);
    blackboard->set("motor_direction_publisher", motorDirectionPublisher);
    blackboard->set("navigation_control", activatePublisher);

    // std_msgs::Float32 positiveAngleControlMsg;
    // positiveAngleControlMsg.data = 60;
    // positiveAnglePublisher.publish(positiveAngleControlMsg);

    // std_msgs::Float32 negativeAngleControlMsg;
    // negativeAngleControlMsg.data = 290;
    // negativeAnglePublisher.publish(negativeAngleControlMsg);

    int cycle1 = 0;
    int cycle2 = 0;
    int cnt = 0;
    int cnnnt = 0;
    while (ros::ok())
    {
    if(cnnnt > 0)
    {
    std_msgs::Int32 navigationControlMsg;
    navigationControlMsg.data = 1;
    activatePublisher.publish(navigationControlMsg);
    }
    ros::Duration(5.0).sleep();
    
    std::string xml_text = R"(
        <root BTCPP_format="3">
        <BehaviorTree ID="ReactiveBehaviorTree">
            <Sequence>
            <RetryUntilSuccessful num_attempts="10000">
                <CheckZValuePositive60 min="60"/>
            </RetryUntilSuccessful>    
                <Sleep duration="1"/>
                <ActivateStop/>
                <Sleep duration="5"/>
                <ReduceLinearActuatorAction value="-25"/>
                <Sleep duration="5"/>
                <TurnOnMotor/>
                <Sleep duration="5"/>
                <IncreaseLinearActuatorAction value="25"/>
                <Sleep duration="5"/>
                <ActivateReverse/>
                <Sleep duration="5"/>
            </Sequence>
        </BehaviorTree>
        </root>)";

    std::cout << "Loaded XML:\n" << xml_text << std::endl;

    auto tree = factory.createTreeFromText(xml_text, blackboard);

    ros::Rate rate(1);
    int tick_count = 0;
    int success_count = 0;
    const int desired_success_count = 7;

    while (success_count < desired_success_count)
    {
        BT::NodeStatus status = tree.tickRoot();
        rate.sleep();
        tick_count++;

        std::cout << "Tick count: " << tick_count << " | Success count: " << success_count << " | Status: " << BT::toStr(status) << std::endl;

        if (status == BT::NodeStatus::SUCCESS)
        {
            success_count++;
            tree.rootNode()->halt();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Behavior Tree execution completed after " << tick_count << " cycles." << std::endl;

    ros::Duration(5.0).sleep();

    ros::Subscriber wireValueSub = nh.subscribe("/wire_value", 1, wireValueCallback);

    std_msgs::Int32 navigationControlMsg;
    navigationControlMsg.data = 0;
    activatePublisher.publish(navigationControlMsg);

    ros::Duration(1.0).sleep();
    std_msgs::Int32 servoPoseMsg;
    servoPoseMsg.data = 65;
    servoPosePublisher.publish(servoPoseMsg);
    ros::Duration(5.0).sleep();

    std_msgs::Int8 motorDirectionMsg;
    motorDirectionMsg.data = 2;
    motorDirectionPublisher.publish(motorDirectionMsg);
    ros::Duration(1.0).sleep();

    while (ros::ok()) {
        ros::Duration(1.0).sleep();

        if (wireValueMsg.data < 0.022) {
            motorDirectionMsg.data = 0;
            motorDirectionPublisher.publish(motorDirectionMsg);
            break;
        }
    }

    ros::Duration(5.0).sleep();
    

    int bash_script_result;
    std::string homeDirectory;
    std::string scriptPath;

    if (cnt == 0) {
        homeDirectory = getHomeDirectory();
        scriptPath = homeDirectory + "/lateral_shift.sh";

        //bash_script_result = std::system(scriptPath.c_str());

        system("bash ~/lateral_shift.sh &");

        ros::Duration(5.0).sleep();

        std_msgs::Int32 lateralShiftMsg;
        lateralShiftMsg.data = 1;
        std::cout << "Publishing lateral shift message with data: " << lateralShiftMsg.data << std::endl;
        lateralShiftPublisher.publish(lateralShiftMsg);

        ros::Duration(50.0).sleep();

        std::system("rosnode kill lateral_shift_controller");
        
    } else {
        homeDirectory = getHomeDirectory();
        scriptPath = homeDirectory + "/lateral_shift.sh";

        //bash_script_result = std::system(scriptPath.c_str());

        system("bash ~/lateral_shift.sh &");

        ros::Duration(5.0).sleep();

        std_msgs::Int32 lateralShiftMsg;
        lateralShiftMsg.data = 1;
        std::cout << "Publishing lateral shift message with data: " << lateralShiftMsg.data << std::endl;
        lateralShiftPublisher.publish(lateralShiftMsg);

        ros::Duration(2.0).sleep();

        std::system("rosnode kill lateral_shift_controller");

        ros::Duration(3.0).sleep();

        scriptPath = homeDirectory + "/lateral_shift.sh";

        //bash_script_result = std::system(scriptPath.c_str());

        system("bash ~/lateral_shift.sh &");

        ros::Duration(5.0).sleep();

        lateralShiftMsg.data = 1;
        std::cout << "Publishing lateral shift message with data: " << lateralShiftMsg.data << std::endl;
        lateralShiftPublisher.publish(lateralShiftMsg);

        ros::Duration(50.0).sleep();

        std::system("rosnode kill lateral_shift_controller");
    }
    cnt++;
    cnnnt++;
    }
    spinner.stop();
    ros::shutdown();
    return 0;
}
