#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "a1_msgs/msg/mode.hpp"
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
enum {
	CMD_SET_MODE_STAND = 0,
	CMD_SET_MODE_FORCE_STAND,
	CMD_SET_MODE_WALK,
	CMD_SET_BODY_HEIGH_UP,
	CMD_SET_BODY_HEIGH_DOWN,
	CMD_SET_YAW_UP,
	CMD_SET_YAW_DOWN,
	CMD_SET_PITCH_UP,
	CMD_SET_PITCH_DOWN,
	CMD_SET_ROLL_LEFT,
	CMD_SET_ROLL_RIGHT,
	CMD_SET_CLEAR_ALL
};
#define KEYCODE_I_CAP  0x49
#define KEYCODE_I  0x69
#define KEYCODE_U_CAP  0x55
#define KEYCODE_U  0x75
#define KEYCODE_R_CAP  0x52
#define KEYCODE_R  0x72
#define KEYCODE_P_CAP  0x50
#define KEYCODE_P  0x70
#define KEYCODE_Y  0x79
#define KEYCODE_Y_CAP  0x59
#define KEYCODE_E  0x65
#define KEYCODE_E_CAP  0x45
#define KEYCODE_Q 0x71
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_H 0x68
#define KEYCODE_H_CAP 0x48
#define KEYCODE_0  0x30
#define KEYCODE_1  0x31
#define KEYCODE_2  0x32
/* shift */
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
int kfd = 0;
struct termios cooked, raw;
using namespace std::chrono_literals;
class PubNode: public rclcpp::Node
{
public:
    PubNode():Node("pub_node"){
        walk_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	 mode_pub = this->create_publisher<a1_msgs::msg::Mode>("cmd_mode", 10);
    }
    void keyboardLoop();
    void help_info();
private:
    void pub_walk(float forwardSpeed, float sideSpeed, float rotateSpeed){
        geometry_msgs::msg::Twist msg;
	 
        msg.linear.x = forwardSpeed;
        msg.linear.y = sideSpeed;
        msg.angular.z = rotateSpeed;
	 RCLCPP_INFO(this->get_logger(), "sending vel (%0.2f) (%0.2f) (%0.2f)",forwardSpeed, sideSpeed, rotateSpeed);
	 walk_pub->publish(msg);
    }
    void pub_command(uint8_t mode, float value){
        a1_msgs::msg::Mode msg;
	 msg.mode = mode;
	 msg.value = value;
	 RCLCPP_INFO(this->get_logger(), "sending cmd mode [%d] value[%0.2f]", mode, value);
	 mode_pub->publish(msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walk_pub;
    rclcpp::Publisher<a1_msgs::msg::Mode>::SharedPtr mode_pub;
};
void PubNode::help_info()
{
	puts("========================");
	puts("Reading from keyboard");
       puts("Use WASD keys to control the robot");
       puts("Press Shift to move faster");
	puts("Press Shift to move faster");
	puts("========================");
}

void PubNode::keyboardLoop()
{
    char c;
    bool dirty = false;
    float forwardSpeed = 0;
    float sideSpeed = 0;
    float rotateSpeed = 0;
    int key_count = 0;
    float set_value = 0;
    help_info();
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    raw.c_lflag &=~ (ICANON | ECHO);

    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        /* get the next event from the keyboard */
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll error");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            { 
                
                perror("read error");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                key_count = 0;
            	  pub_walk(0, 0, 0);
            	  for (int i = 0; i < 5; i++)
		  	pub_command(CMD_SET_CLEAR_ALL, 0);
                dirty = false;
            }
            continue;
        }
        std::cout << c << std::endl;
	 std::cout << std::hex << int(c) << std::endl;
        switch(c)
        {
            case KEYCODE_W:
                forwardSpeed = 0.1;
		   sideSpeed      = 0;
                rotateSpeed    = 0;
                dirty = true;
		   pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_S:
                forwardSpeed = -0.1;
		   sideSpeed      = 0;
                rotateSpeed    = 0;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_A:
                forwardSpeed = 0;
		   sideSpeed      = 0.1;
                rotateSpeed    = 0;
                dirty = true;
		   pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_D:
                forwardSpeed = 0;
		   sideSpeed      = -0.1;
                rotateSpeed    = 0;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;            
				
            case KEYCODE_W_CAP:
                forwardSpeed = 1;
                sideSpeed = 0;
		  rotateSpeed = 0;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_S_CAP:
                forwardSpeed = -1;
                sideSpeed = 0;
		  rotateSpeed = 0;
		  dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_A_CAP:
                forwardSpeed = 0;
                sideSpeed = 1;
		  rotateSpeed = 0;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;
            case KEYCODE_D_CAP:
                forwardSpeed = 0;
                sideSpeed = -1;
		  rotateSpeed = 0;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
                break;    
	     case KEYCODE_Q:
		  forwardSpeed = 0;
                sideSpeed = 0;
		  rotateSpeed = 0.5;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
		  break;
	     case KEYCODE_E:
		  forwardSpeed = 0;
                sideSpeed = 0;
		  rotateSpeed = -0.5;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
		  break;
	     case KEYCODE_Q_CAP:
		  forwardSpeed = 0;
                sideSpeed = 0;
		  rotateSpeed = 1;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
		  break;
	     case KEYCODE_E_CAP:
		  forwardSpeed = 0;
                sideSpeed = 0;
		  rotateSpeed = -1;
                dirty = true;
		  pub_walk(forwardSpeed, sideSpeed, rotateSpeed);
		  break;
	     case KEYCODE_0:
		  dirty = true;
		  pub_command(CMD_SET_MODE_STAND, 0);
		  break;
	     case KEYCODE_1:
		  dirty = true;
		  pub_command(CMD_SET_MODE_FORCE_STAND, 0);
		  break;
            case KEYCODE_2:
		  dirty = true;
		  pub_command(CMD_SET_MODE_WALK, 0);
		  break;
	     case KEYCODE_U://pitch
	         dirty = false;
	         key_count++;
		  set_value = key_count * (-0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_PITCH_UP, set_value);
		  break;
	     case KEYCODE_U_CAP://pitch
		  dirty = false;
	         key_count++;
		  set_value = key_count * (-0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_PITCH_DOWN, set_value);
		  break;
	     case KEYCODE_Y://yaw
		  dirty = false;
	         key_count++;
		  set_value = key_count * (0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_YAW_UP, set_value);
		  break;
	     case KEYCODE_Y_CAP://yaw
		  dirty = false;
	         key_count++;
		  set_value = key_count * (-0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_YAW_DOWN, set_value);
		  break;
	     case KEYCODE_H://bodyHeight
		  dirty = false;
	         key_count++;
		  set_value = key_count * (0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_BODY_HEIGH_UP, set_value);
		  break;
	     case KEYCODE_H_CAP://bodyHeight
		  dirty = false;
	         key_count++;
		  set_value = key_count * (-0.25);
		  if (key_count > 3)
		  {
		  	key_count = 0;
			dirty = true;
			break;
		  }
		  pub_command(CMD_SET_BODY_HEIGH_DOWN, set_value);
		  break;
#if 0
	     case KEYCODE_R://roll
		  dirty = true;
		  pub_command(CMD_SET_ROLL_LEFT, 0);
		  break;
	     case KEYCODE_R_CAP://roll
		  dirty = true;
		  pub_command(CMD_SET_ROLL_RIGHT, 0);
		  break;
#endif
            default:
                dirty = false;
        }
    }
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	
    PubNode tbk;
    /*create pthread monitor keyboard*/
    boost::thread t = boost::thread(boost::bind(&PubNode::keyboardLoop, &tbk));
    auto node = std::make_shared<PubNode>();
	
    rclcpp::spin(node);
    rclcpp::shutdown();
	
    t.interrupt();
    t.join();
    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}

