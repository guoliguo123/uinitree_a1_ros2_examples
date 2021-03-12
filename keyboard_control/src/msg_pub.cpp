#include "keyboard.h"
#include"rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "a1_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
void PubNode::publisherWalk(float forwardSpeed, float sideSpeed, float rotateSpeed){
	geometry_msgs::msg::Twist msg;
	msg.linear.x = forwardSpeed;
	msg.linear.y = sideSpeed;
	msg.angular.z = rotateSpeed;
	RCLCPP_INFO(this->get_logger(), "sending vel (%0.2f) (%0.2f) (%0.2f)",forwardSpeed, sideSpeed, rotateSpeed);
	walk_pub->publish(msg);
}
void PubNode::publisherCmd(uint8_t mode, float value){
	a1_msgs::msg::Mode msg;
	msg.mode = mode;
	msg.value = value;
	RCLCPP_INFO(this->get_logger(), "sending cmd mode [%d] value[%0.2f]", mode, value);
	mode_pub->publish(msg);
}

bool PubNode::setValue(float *set_value, float base_val)
{
	bool dirty = false;
	static int key_count = 0;
	key_count++;
	*set_value = key_count * base_val;
	if (key_count > 3)
	{
		key_count = 0;
		dirty = true;
	}
	return dirty;
}
void PubNode::publisherReset()
{
	publisherWalk(0, 0, 0);
	for (int i = 0; i < 5; i++)
		publisherCmd(CMD_SET_CLEAR_ALL, 0);
}
bool PubNode::keyboardPressProcess(char ch)
{
	bool dirty = false; 
	float set_value = 0;
	
	switch (ch)
	{
		case KEYCODE_W:
			dirty = true;
			publisherWalk(0.1, 0, 0);
			break;
		case KEYCODE_S:
			dirty = true;
			publisherWalk(-0.1, 0, 0);
			break;
		case KEYCODE_A:
			dirty = true;
			publisherWalk(0, 0.1, 0);
			break;
		case KEYCODE_D:
			dirty = true;
			publisherWalk(0, -0.1, 0);
			break;
		case KEYCODE_SHIFT_S:
			dirty = true;
			publisherWalk(-1, 0, 0);
			break;
		case KEYCODE_SHIFT_A:
			dirty = true;
			publisherWalk(0, 1, 0);
			break;
		case KEYCODE_SHIFT_D:
			dirty = true;
			publisherWalk(0, -1, 0);
			break;
		case KEYCODE_SHIFT_W:
			dirty = true;
			publisherWalk(1, 0, 0);
			break;
		case KEYCODE_Q:
			dirty = true;
			publisherWalk(0, 0, 0.5);
			break;
		case KEYCODE_E:
			dirty = true;
			publisherWalk(0, 0, -0.5);
			break;
		case KEYCODE_SHIFT_Q:
			dirty = true;
			publisherWalk(0, 0, 1);
			break;
		case KEYCODE_SHIFT_E:
			dirty = true;
			publisherWalk(0, 0, -1);
			break;
		case KEYCODE_0:
			dirty = true;
			publisherCmd(CMD_SET_MODE_STAND, 0);
			break;
		case KEYCODE_1:
			dirty = true;
			publisherCmd(CMD_SET_MODE_FORCE_STAND, 0);
			break;
		case KEYCODE_2:
			dirty = true;
			publisherCmd(CMD_SET_MODE_WALK, 0);
			break;
		case KEYCODE_U:
			dirty = setValue(&set_value, 0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_PITCH_UP, set_value);
			break;
		case KEYCODE_SHIFT_U:
			dirty = setValue(&set_value, -0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_PITCH_DOWN, set_value);
			break;
		case KEYCODE_Y:
			dirty = setValue(&set_value, 0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_YAW_UP, set_value);
			break;
		case KEYCODE_SHIFT_Y:
			dirty = setValue(&set_value, -0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_YAW_DOWN, set_value);
			break;
		case KEYCODE_H:
			dirty = setValue(&set_value, 0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_BODY_HEIGH_UP, set_value);
			break;
		case KEYCODE_SHIFT_H:
			dirty = setValue(&set_value, -0.25);
			if (dirty == true)
				break;
			publisherCmd(CMD_SET_BODY_HEIGH_DOWN, set_value);
			break;
		case KEYCODE_R:
			dirty = true;
			publisherCmd(CMD_SET_ROLL_LEFT, 0);
			break;
		case KEYCODE_SHIFT_R:
			dirty = true;
			publisherCmd(CMD_SET_ROLL_RIGHT, 0);
			break;
		default:
			break;
	}
	return dirty;
}