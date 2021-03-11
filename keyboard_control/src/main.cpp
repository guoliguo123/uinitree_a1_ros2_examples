#include "keyboard.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string node_name = "keyboard";
    KeyBoard keyboard;
    /*create pthread monitor keyboard*/
    boost::thread t = boost::thread(boost::bind(&KeyBoard::keyboardLoop, &keyboard));
   
    auto node = rclcpp::Node::make_shared(node_name);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    t.interrupt();
    t.join();
    keyboard.exitKeyBoardLoop();
    return 0;
}
