#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarrayvel.hpp>
#include <ros/console.h>


class CartesianToJointVelocity {
public:
    CartesianToJointVelocity(ros::NodeHandle& nh) {

        std::cout << "Initializing CartesianToJointVelocity node..." << std::endl;
        if (!kdl_parser::treeFromParam("robot_description", kdl_tree_)) {
            std::cout << "Failed to parse kdl tree from xml" << std::endl;
            return;
        }

        if (!kdl_tree_.getChain("panda_link0", "panda_link8", kdl_chain_)) {
            std::cout << "Failed to get KDL chain" << std::endl;
            return;
        }

        ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);

        sub_cartesian_velocity_ = nh.subscribe("/franka/ee_target_velocity", 1, &CartesianToJointVelocity::cartesianVelocityCallback, this);
        pub_joint_velocity_ = nh.advertise<sensor_msgs::JointState>("/franka/desired_joint_velocity", 1);
        //sub_current_joint_velocity = nh.subscribe("joint_velocity_command", 1, currentVelocityCallback);
        std::cout << "Subscribers and publishers created successfully." << std::endl;
       
    }

    void cartesianVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO("Received cartesian velocity callback.");
        KDL::Twist desired_velocity;
        desired_velocity.vel.data[0] = msg->linear.x;
        desired_velocity.vel.data[1] = msg->linear.y;
        desired_velocity.vel.data[2] = msg->linear.z;
        desired_velocity.rot.data[0] = msg->angular.x;
        desired_velocity.rot.data[1] = msg->angular.y;
        desired_velocity.rot.data[2] = msg->angular.z;

        KDL::JntArrayVel joint_velocities(kdl_chain_.getNrOfJoints());

        if (ik_vel_solver_->CartToJnt(KDL::JntArray(kdl_chain_.getNrOfJoints()), desired_velocity, joint_velocities.qdot) >= 0) {
            sensor_msgs::JointState joint_state_msg;
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.name = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7" };
            joint_state_msg.velocity.resize(kdl_chain_.getNrOfJoints());

            for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
                joint_state_msg.velocity[i] = joint_velocities.qdot.data[i];
            }

            pub_joint_velocity_.publish(joint_state_msg);
        }
        else {
            ROS_WARN("Failed to solve for joint velocities");
        }

        ROS_INFO("Received Target Cartesian Velocity: linear(%f, %f, %f), angular(%f, %f, %f)",
            msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z);
    }

private:
    ros::Subscriber sub_cartesian_velocity_;
    ros::Subscriber sub_current_joint_velocity;
    ros::Publisher pub_joint_velocity_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cartesian_to_joint_velocity");
    ros::NodeHandle nh;
    CartesianToJointVelocity cartesian_to_joint_velocity(nh);
    ros::spin();
    return 0;
}
