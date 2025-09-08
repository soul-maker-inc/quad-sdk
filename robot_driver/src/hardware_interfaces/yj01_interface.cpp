#include "robot_driver/hardware_interfaces/yj01_interface.hpp"

Yj01Interface::Yj01Interface() : HardwareInterface() {}

void Yj01Interface::loadInterface(int argc, char **argv) {}

void Yj01Interface::unloadInterface() {}

bool Yj01Interface::send(
    const quad_msgs::msg::LegCommandArray &last_leg_command_array_msg,
    const Eigen::VectorXd &user_tx_data) {

  return true;
}

bool Yj01Interface::recv(sensor_msgs::msg::JointState &joint_state_msg,
                         sensor_msgs::msg::Imu &imu_msg,
                         Eigen::VectorXd &user_rx_data) {

  return true;
}
