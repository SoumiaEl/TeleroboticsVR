#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class FrankaPandaControllerNode : public rclcpp::Node
{
public:
    FrankaPandaControllerNode()
    : Node("franka_panda_controller_node")
    {
        // Initialization
        desired_joint_states_callback_ = false;
        teleoperation_mode_ = this->declare_parameter("teleoperation_mode", 1);
        RCLCPP_INFO(this->get_logger(), "Joint Command: %d",teleoperation_mode_);
        robot_namespace_ = this->declare_parameter("robot_name", std::string("franka"));

        // Subscriptions
        desired_jointstate_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            robot_namespace_ + "/desired_joint_state", 
            10, 
            std::bind(&FrankaPandaControllerNode::DesiredJointStateCallback, this, std::placeholders::_1)
        );

        // Publishers
        joint_pos_cmd_pub_[0] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint1_position_controller/command", 10);
        joint_pos_cmd_pub_[1] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint2_position_controller/command", 10);
        joint_pos_cmd_pub_[2] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint3_position_controller/command", 10);
        joint_pos_cmd_pub_[3] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint4_position_controller/command", 10);
        joint_pos_cmd_pub_[4] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint5_position_controller/command", 10);
        joint_pos_cmd_pub_[5] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint6_position_controller/command", 10);
        joint_pos_cmd_pub_[6] = this->create_publisher<std_msgs::msg::Float64>(robot_namespace_+"/joint7_position_controller/command", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&FrankaPandaControllerNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "franka_panda_controller_node start!..\n");
    }

private:
    void DesiredJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        desired_joint_states_callback_ = true;

        DesiredJointState_.position = msg->position;
        DesiredJointState_.velocity = msg->velocity;
        DesiredJointState_.effort = msg->effort;
    }

    void timer_callback()
    {
        std_msgs::msg::Float64 joint_pos_cmd[7];

        if(desired_joint_states_callback_){
            // Teleoperation
            for(int i = 0; i < 7; i++){
                joint_pos_cmd[i].data = DesiredJointState_.position[i+2];
                joint_pos_cmd_pub_[i]->publish(joint_pos_cmd[i]);
            }
        }
        else
        {
            double default_positions[7] = {0.0, -0.7, 0, -1.8, 0, 1.1, 0.0};
            for(int i = 0; i < 7; i++){
                joint_pos_cmd[i].data = default_positions[i];
                joint_pos_cmd_pub_[i]->publish(joint_pos_cmd[i]);
            }
        }
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_jointstate_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_pos_cmd_pub_[7];

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    sensor_msgs::msg::JointState DesiredJointState_;
    bool desired_joint_states_callback_;
    int teleoperation_mode_;
    std::string robot_namespace_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankaPandaControllerNode>());
    rclcpp::shutdown();
    return 0;
}
