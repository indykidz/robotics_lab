
#ifndef ARM_DESCRIPTION_ARMCONTROLLER_H
#define ARM_DESCRIPTION_ARMCONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


class ArmControllerNode {
public:
    /*!
    * Constructor.
    * @param nodeHandle the ROS node handle.
    */
    ArmControllerNode(ros::NodeHandle& nodeHandle);
    /*!
    * Destructor.
    */
    virtual ~ArmControllerNode();
private:

/*!
* ROS topic callback method.
* @param message the received message.
*/
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;
    //! ROS topic subscriber.
    ros::Subscriber jointStateSub;
    //! Ros topic publishers.
    ros::Publisher joint0_command_pub_;
    ros::Publisher joint1_command_pub_;
    ros::Publisher joint2_command_pub_;
    ros::Publisher joint3_command_pub_;
};


#endif //ARM_DESCRIPTION_ARMCONTROLLER_H
