#include"rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "a1_msgs/msg/mode.hpp"
class PubNode: public rclcpp::Node
{
public:
    PubNode():Node("pub_node"){
        walk_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	 mode_pub = this->create_publisher<a1_msgs::msg::Mode>("cmd_mode", 10);
    }
    //void keyboardPressProcess(char c); 
//private:
    void pub_walk(float forwardSpeed, float sideSpeed, float rotateSpeed);
    void pub_command(uint8_t mode, float value);
    bool keyboardPressProcess(char ch);
    void pub_clear();
    //int pub_set_value();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walk_pub;
    rclcpp::Publisher<a1_msgs::msg::Mode>::SharedPtr mode_pub;
};

