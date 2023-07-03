#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/tree.hpp"
#include "kdl/rigidbodyinertia.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/jacobian.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_kdl/tf2_kdl.h>

class FrankaKinematicsSolver : public rclcpp::Node
{
public:
    FrankaKinematicsSolver();

private:
    geometry_msgs::msg::PoseStamped target_ee_pose_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr updated_ee_pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr franka_jointstate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_ee_pose_sub_;

    std::string robot_name_space_;
    std::string robot_name_;

    bool JointStateGet_{false};

    KDL::Chain franka_chain_;

    sensor_msgs::msg::JointState current_jointstate_;

    int teleoperation_mode_;

    void TargetEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    int KDLForwardKinematics(const KDL::Chain robot_chain, const KDL::JntArray joint_in, KDL::Frame &cart_pos_out);
    int KDLInverseKinematics(const KDL::Chain robot_chain, const KDL::Frame target_frame,
                             const KDL::JntArray q_current, KDL::JntArray &q_out);
    void KDLFrameToPoseStampedMsg(const KDL::Frame &kdl_frame_in, geometry_msgs::msg::PoseStamped &pose_stamped_out);
    void PoseStampedMsgToKDLFrame(const geometry_msgs::msg::PoseStamped &pose_stamped_in, KDL::Frame &kdl_frame_out);
    void KDLJntArrayToJointStateMsg(const KDL::JntArray &jnt_array_in, sensor_msgs::msg::JointState &jointstate_out);
    void JointStateMsgToKDLJntArray(const sensor_msgs::msg::JointState &jointstate_in, KDL::JntArray &jnt_array_out);
    void CurrentJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

FrankaKinematicsSolver::FrankaKinematicsSolver()
    : Node("franka_kinematics_solver_node"),
      
      JointStateGet_(false)

{
    // Declare parameters
    this->declare_parameter("robot_name_space", "franka");
    this->declare_parameter("robot_name", "panda");
    this->declare_parameter("teleoperation_mode", 1);

    // Initialize member variables
    robot_name_space_ = this->get_parameter("robot_name_space").as_string();
    robot_name_ = this->get_parameter("robot_name").as_string();
    teleoperation_mode_ = this->get_parameter("teleoperation_mode").as_int();


    RCLCPP_INFO(this->get_logger(), "Teleoperation Mode: %d", teleoperation_mode_);

    desired_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(robot_name_space_ + "/desired_joint_state", 10);
    updated_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/updated_target_ee_pose", 10);
    target_ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("ee_target_pose", 10, std::bind(&FrankaKinematicsSolver::TargetEEPoseCallback, this, std::placeholders::_1));
    franka_jointstate_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(robot_name_space_ + "/joint_states", 10, std::bind(&FrankaKinematicsSolver::CurrentJointStateCallback, this, std::placeholders::_1));

    // Get KDL_tree information from ROS parameter server
    KDL::Tree franka_kdl_tree;
    std::string robot_desc_string;
    this->declare_parameter("robot_description");
    if (!this->get_parameter("robot_description", robot_desc_string))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get robot_description parameter");
        return; 
    }

    if (!kdl_parser::treeFromString(robot_desc_string, franka_kdl_tree))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully construct kdl tree from robot_description");
    }

    unsigned int num_joints = franka_kdl_tree.getNrOfJoints();
    RCLCPP_INFO(this->get_logger(), "Number of joints: %d\n", num_joints);

    // Chain Creation
    bool ret0 = franka_kdl_tree.getChain("panda_link0", "panda_hand", franka_chain_);
    RCLCPP_INFO(this->get_logger(), "GetChain Result: %d", ret0);
    // You might need to do additional work to initialize the KDL::Chain franka_chain_
}

void FrankaKinematicsSolver::TargetEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    target_ee_pose_ = *msg; // Target End-effector Pose.

    if (!JointStateGet_)
        return;

    sensor_msgs::msg::JointState current_joint_state;

    // Get current franka joint state information for joint1 to joint7
    // jointstate's [0],[1] data is finger joints, and we don't need it
    for (unsigned int i = 0; i < 7; i++)
    {
        current_joint_state.name.push_back(current_jointstate_.name[i + 2]);
        current_joint_state.position.push_back(current_jointstate_.position[i + 2]);
        current_joint_state.velocity.push_back(current_jointstate_.velocity[i + 2]);
        current_joint_state.effort.push_back(current_jointstate_.effort[i + 2]);
    }

    // Get Current joint angle q_current by current_joint_state
    KDL::JntArray q_current;
    JointStateMsgToKDLJntArray(current_joint_state, q_current);

    // Set target frame (Pose of End-Effector in Cartesian frame)
    KDL::Frame target_frame;
    PoseStampedMsgToKDLFrame(target_ee_pose_, target_frame);

    // Compute target JointArray q_target
    KDL::JntArray q_target(q_current.rows());
    int IK_Result = KDLInverseKinematics(franka_chain_, target_frame, q_current, q_target);
    if (IK_Result != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Inverse Kinematics not Solved : %d", IK_Result);
        return;
    }

    // Convert JointArray to JointStateMsg
    sensor_msgs::msg::JointState desired_joint_state;
    KDLJntArrayToJointStateMsg(q_target, desired_joint_state);

    // Publish final joint state.
    sensor_msgs::msg::JointState final_joint_state; // Including finger joints

    final_joint_state = current_jointstate_;

    // Updating desired joint positions
    for (unsigned int i = 0; i < 7; i++)
    {
        final_joint_state.name[i + 2] = desired_joint_state.name[i];
        final_joint_state.position[i + 2] = desired_joint_state.position[i];
        final_joint_state.velocity[i + 2] = desired_joint_state.velocity[i];
        final_joint_state.effort[i + 2] = desired_joint_state.effort[i];
    }

    final_joint_state.header.stamp = this->now();
    desired_joint_state_pub_->publish(final_joint_state);

    // Forward Kinematics To check the result
    geometry_msgs::msg::PoseStamped updated_ee_pose;
    KDL::JntArray q_now;

    JointStateMsgToKDLJntArray(desired_joint_state, q_now);

    KDL::Frame ee_frame;
    // unused : int fk_res = KDLForwardKinematics(franka_chain_, q_now, ee_frame);
    KDLFrameToPoseStampedMsg(ee_frame, updated_ee_pose);
    updated_ee_pose.header.frame_id = "panda_link0";
    updated_ee_pose.header.stamp = msg->header.stamp;

    updated_ee_pose_pub_->publish(updated_ee_pose);
}

int FrankaKinematicsSolver::KDLForwardKinematics(const KDL::Chain robot_chain, const KDL::JntArray joint_in, KDL::Frame &cart_pos_out)
{
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot_chain);
    int fk_result = fksolver.JntToCart(joint_in, cart_pos_out); // if < 0 something went wrong
    return fk_result;
}

int FrankaKinematicsSolver::KDLInverseKinematics(const KDL::Chain franka_chain_, const KDL::Frame target_frame,
                                                 const KDL::JntArray q_current, KDL::JntArray &q_out)
{
    // Joint Limit
    KDL::JntArray q_min(q_current.rows());
    KDL::JntArray q_max(q_current.rows());

    // Set Joint Limit for each joint (Below is franka joint limits)
    q_min(0) = -2.8973;
    q_max(0) = 2.8973;
    q_min(1) = -1.7628;
    q_max(1) = 1.7628;
    q_min(2) = -2.8973;
    q_max(2) = 2.8973;
    q_min(3) = -3.0718;
    q_max(3) = 3.0718;
    q_min(4) = -2.8973;
    q_max(4) = 2.8973;
    q_min(5) = -0.0175;
    q_max(5) = 3.7525;
    q_min(6) = -2.8973;
    q_max(6) = 2.8973;

    // Creation of the solvers
    KDL::ChainFkSolverPos_recursive fksolver(franka_chain_);                                                        // Forward position solver
    KDL::ChainIkSolverVel_pinv iksolver_vel(franka_chain_);                                                         // Inverse velocity solver
    KDL::ChainIkSolverPos_NR_JL iksolver_pos_nr_jl(franka_chain_, q_min, q_max, fksolver, iksolver_vel, 200, 1e-3); // Maximum 100 iterations, stop at accuracy 1e-3

    // Solving IK
    int ik_result = iksolver_pos_nr_jl.CartToJnt(q_current, target_frame, q_out);
    return ik_result;
}

void FrankaKinematicsSolver::KDLFrameToPoseStampedMsg(const KDL::Frame &kdl_frame_in, geometry_msgs::msg::PoseStamped &pose_stamped_out)
{
    pose_stamped_out.pose.position.x = kdl_frame_in.p.x();
    pose_stamped_out.pose.position.y = kdl_frame_in.p.y();
    pose_stamped_out.pose.position.z = kdl_frame_in.p.z();

    double qx, qy, qz, qw;
    kdl_frame_in.M.GetQuaternion(qx, qy, qz, qw);

    pose_stamped_out.pose.orientation.x = qx;
    pose_stamped_out.pose.orientation.y = qy;
    pose_stamped_out.pose.orientation.z = qz;
    pose_stamped_out.pose.orientation.w = qw;
}

void FrankaKinematicsSolver::PoseStampedMsgToKDLFrame(const geometry_msgs::msg::PoseStamped &pose_stamped_in, KDL::Frame &kdl_frame_out)
{
    double ee_roll, ee_pitch, ee_yaw;
    tf2::Quaternion quaternion(
        pose_stamped_in.pose.orientation.x,
        pose_stamped_in.pose.orientation.y,
        pose_stamped_in.pose.orientation.z,
        pose_stamped_in.pose.orientation.w);

    tf2::Matrix3x3 mat(quaternion);
    mat.getRPY(ee_roll, ee_pitch, ee_yaw);

    double ee_x = pose_stamped_in.pose.position.x;
    double ee_y = pose_stamped_in.pose.position.y;
    double ee_z = pose_stamped_in.pose.position.z;

    kdl_frame_out = KDL::Frame(KDL::Rotation::RPY(ee_roll, ee_pitch, ee_yaw), KDL::Vector(ee_x, ee_y, ee_z));
}

void FrankaKinematicsSolver::KDLJntArrayToJointStateMsg(const KDL::JntArray &jnt_array_in, sensor_msgs::msg::JointState &jointstate_out)
{
    for (unsigned int i = 0; i < jnt_array_in.rows(); i++)
    {
        jointstate_out.position.push_back(jnt_array_in(i));
    }
    jointstate_out.name.resize(jnt_array_in.rows());
    jointstate_out.velocity.resize(jnt_array_in.rows());
    jointstate_out.effort.resize(jnt_array_in.rows());
}

void FrankaKinematicsSolver::JointStateMsgToKDLJntArray(const sensor_msgs::msg::JointState &jointstate_in, KDL::JntArray &jnt_array_out)
{
    jnt_array_out.resize(jointstate_in.position.size());
    for (unsigned int i = 0; i < jointstate_in.position.size(); i++)
    {
        jnt_array_out(i) = jointstate_in.position[i];
    }
}
void FrankaKinematicsSolver::CurrentJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    JointStateGet_ = true;
    current_jointstate_ = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto franka_kinematics = std::make_shared<FrankaKinematicsSolver>();

    rclcpp::spin(franka_kinematics);

    rclcpp::shutdown();
    return 0;
}
