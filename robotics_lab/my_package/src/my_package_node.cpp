#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_package/my_msg.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_package_node");
	ros::NodeHandle n;
	
	ros::Publisher my_package_pub = n.advertise<my_package::my_msg>("my_package_topic", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	
	
	int param;
	if (n.getParam("my_int_param", param))
		ROS_INFO("my_int_param is %d", param);
	else
		ROS_INFO("my_int_param not defined");
	
	my_package::my_msg msg;
	msg.name = "my_msg_name";
	while (ros::ok())
	{
		msg.data = count;
		ROS_INFO("%d", msg.data);
		
		my_package_pub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
