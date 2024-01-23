#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "arm_controller/arm_controller.h"
#include <std_msgs/Float64.h>

class ArmControllerNode {
public:
    ArmControllerNode(ros::NodeHandle& nh) : nh_(nh) {
        joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ArmControllerNode::jointStateCallback, this);
        joint0_command_pub_ = nh_.advertise<std_msgs::Float64>("/PositionJointInterface_J0_controller/command", 10);
        joint1_command_pub_ = nh_.advertise<std_msgs::Float64>("/PositionJointInterface_J1_controller/command", 10);
        joint2_command_pub_ = nh_.advertise<std_msgs::Float64>("/PositionJointInterface_J2_controller/command", 10);
        joint3_command_pub_ = nh_.advertise<std_msgs::Float64>("/PositionJointInterface_J3_controller/command", 10);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
        // Print the current joint positions
        for (size_t i = 0; i < joint_state->name.size(); ++i) {
            ROS_INFO("Joint %s: Position %f", joint_state->name[i].c_str(), joint_state->position[i]);
        }
        float values[4] = {0.1, 0.01, -0.01, -0.1};
    	std_msgs::Float64 Msg[4];
    	for (size_t i = 0; i < joint_state->name.size(); i++) {
        Msg[i].data = joint_state->position[i]+values[i]; 

	joint0_command_pub_.publish(Msg[0]); 
    	joint1_command_pub_.publish(Msg[1]); 
    	joint2_command_pub_.publish(Msg[2]); 
    	joint3_command_pub_.publish(Msg[3]); 
    }
  }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint0_command_pub_;
    ros::Publisher joint1_command_pub_;
    ros::Publisher joint2_command_pub_;
    ros::Publisher joint3_command_pub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    ArmControllerNode arm_controller_node(nh);

    ros::spin();

    return 0;
}
