#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class ArmControllerNode {
public:
    ArmControllerNode(ros::NodeHandle& nh) : nh_(nh) {
        joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ArmControllerNode::jointStateCallback, this);
        joint_command_pub_ = nh_.advertise<std_msgs::Float64>("/joint_position_controller/command", 10);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
        // Print the current joint positions
        for (size_t i = 0; i < joint_state->name.size(); ++i) {
            ROS_INFO("Joint %s: Position %f", joint_state->name[i].c_str(), joint_state->position[i]);
        }

        // Example: Publish the position of the first joint as the joint command
        if (!joint_state->position.empty()) {
            std_msgs::Float64 joint_command;
            joint_command.data = joint_state->position[0];  // Use the position of the first joint
            joint_command_pub_.publish(joint_command);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    ArmControllerNode arm_controller_node(nh);

    ros::spin();

    return 0;
}
