#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

std::vector<double> joints_last_position {0, 0};
bool move_state = false;
ros::ServiceClient client;

void move_arm_center() {
	simple_arm::GoToPosition srv;
	srv.request.joint_1 = 1.57;
	srv.request.joint_2 = 1.57;
	
	if (!client.call(srv)) {
		ROS_ERROR("failed to call safe move");
	}
}

void joint_states_callback(const sensor_msgs::JointState js) {
	std::vector<double> cur_position = js.position;

	double tolerance = 0.0005;

	if (cur_position[0] - joints_last_position[0] < tolerance
		 || cur_position[1] - joints_last_position[1] < tolerance) {
		move_state = false;
	} else {
		move_state = true;
		joints_last_position = cur_position;
	}
}

void look_away_callback(const sensor_msgs::Image img) {
	bool uniform_image = true;

	for (int i = 0; i < img.height * img.step; i++) {
		if (img.data[i] - img.data[0] != 0) {
			uniform_image = false;
			break;
		}
	}

	if (uniform_image && !move_state) {
		move_arm_center();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "look_way");
	ros::NodeHandle n;

	client = n.serviceClient<simple_arm::GoToPosition>("arm_mover/safe_move");

	ros::Subscriber sub1 = n.subscribe("simple_arm/joint_states", 10, joint_states_callback);
	ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, look_away_callback);
	
	ros::spin();	

	return 0;
}


