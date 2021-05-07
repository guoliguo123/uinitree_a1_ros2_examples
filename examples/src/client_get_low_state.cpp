#include <memory>
#include <chrono>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "a1_comm/A1_comm.h"
#include "a1_msgs/srv/low_state.hpp"

using namespace std::chrono_literals;

class ClientNode     {
public:
    ClientNode() : count_(5) {
        node = rclcpp::Node::make_shared("LowState");
        client = node->create_client<a1_msgs::srv::LowState>(ROS2_SERVICE_GET_LOW_STATE_MSG);
    }
    void client_node_get_low_state();
private:
    rclcpp::Client<a1_msgs::srv::LowState>::SharedPtr client;
    std::shared_ptr<rclcpp::Node> node;
    size_t count_;
};

void ClientNode::client_node_get_low_state() {
    size_t times = 0;
    auto request = std::make_shared<a1_msgs::srv::LowState::Request>();
    while (!client->wait_for_service(1s)) {
        times++;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
        if (count_ == times)
            return;
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto LowState = result.get();
        int i;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "##############ROBOT BASIC INFO#######################");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "levelFlag: %12u     commVersion: %-10u", LowState->levelflag, LowState->commversion);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robotID: %-14u      SN: %-19u", LowState->robotid, LowState->sn);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "bandWidth: %-12u", LowState->bandwidth);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "##############ROBOT IMU INFO#######################");
        for (i = 0; i < UNITREE_A1_IMU_QUATERNION; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "imu.quaternion[%d]: %23f", i, LowState->imu.quaternion[i]);
        }

        for (i = 0; i < UNITREE_A1_IMU_ANGULAR_VELOCITY; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "imu.gyroscope[%d]: %23f", i, LowState->imu.gyroscope[i]);
        }

        for (i = 0; i < UNITREE_A1_IMU_ACCELEROMETER; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "imu.gyroscope[%d]: %23f", i, LowState->imu.accelerometer[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "imu.temperature: %23d", int(LowState->imu.temperature));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "#################ROBOT FOUR LEGS INFO####################");
        for (int i = 0; i < UNITREE_A1_MOTOR_STATE_ELEMENT_NUMS; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].mode: %23f", i, LowState->motorstate[i].mode);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].q: %23f", i, LowState->motorstate[i].q);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].dq: %23f", i, LowState->motorstate[i].dq);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].ddq: %23f", i, LowState->motorstate[i].ddq);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].tauEst: %23f", i, LowState->motorstate[i].tauest);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].q_raw: %23f", i, LowState->motorstate[i].q_raw);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].dq_raw: %23f", i, LowState->motorstate[i].dq_raw);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].ddq_raw: %23f", i, LowState->motorstate[i].ddq_raw);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].temperature: %23f", i, LowState->motorstate[i].temperature);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].reserve[0]: %23f", i, LowState->motorstate[i].reserve[0]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motorstate[%d].reserve[1]: %23f", i, LowState->motorstate[i].reserve[1]);
        }

        for (int i = 0; i < UNITREE_A1_DOG_LEGS; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "footforce[%d]: %23f", i, LowState->footforce[i]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "footForceEst[%d]: %23f", i, LowState->footforceest[i]);
        }
    } 
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
    return;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    ClientNode client;
    //rclcpp::WallRate loop_rate(10.0);
    //while (rclcpp::ok())
    {
        client.client_node_get_low_state();
        //loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

