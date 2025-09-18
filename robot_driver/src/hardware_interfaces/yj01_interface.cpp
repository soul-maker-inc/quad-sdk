#include "robot_driver/hardware_interfaces/yj01_interface.hpp"

Yj01Interface::Yj01Interface() : HardwareInterface(), m_pRecvThread(NULL) {}

void Yj01Interface::loadInterface(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"), "[loadInterface] start");
  if (!m_link.InitUSB()) {
    RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"),
                "[loadInterface] init usb failed");
  }
  if (!m_link.OpenDevice(0)) {
    RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"),
                "[loadInterface] open dev 0 failed");
  }
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"),
              "[loadInterface] initing can ports & motors");

  for (int i = 0; i < 4; ++i) {
    m_link.OpenPort(i);
    for (int j = 0; j < 3; ++j) {
      // setup joint motors
      int jointCanId = 3 * i + j;
      Motor *m = m_link.NewMotor(i, jointCanId);
      m->setupRobStrideRS02();
      m_link.EnableMotor(jointCanId);
      m_link.StartReporting(jointCanId);
    }
    // setup wheel motor
    int wheelCanId = 12 + i;
    Motor *w = m_link.NewMotor(i, wheelCanId);
    w->setupRobStrideRS04();
    m_link.EnableMotor(wheelCanId);
    m_link.StartReporting(wheelCanId);
  }
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"),
              "[loadInterface] start listening can ports");
  // m_pRecvThread = new std::thread(recvProc, &m_link);
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"), "[loadInterface] end");
}

void Yj01Interface::unloadInterface() {
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"), "[unloadInterface] start");
  for (int i = 0; i < 4; ++i) {
    m_link.ClosePort(i);
  }
  m_link.CloseDevice();
  m_link.DeinitUSB();

  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"),
              "[unloadInterface] waiting for listening thread");
  if (m_pRecvThread) {
    m_pRecvThread->join();
    delete m_pRecvThread;
    m_pRecvThread = NULL;
  }
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"), "[unloadInterface] end");
}

bool Yj01Interface::send(
    const quad_msgs::msg::LegCommandArray &last_leg_command_array_msg,
    const Eigen::VectorXd &user_tx_data) {

  bool restart_flag = (user_tx_data[0] == 1);

  for (int i = 0; i < 4; ++i) { // For each leg
    quad_msgs::msg::LegCommand leg_command =
        last_leg_command_array_msg.leg_commands.at(i);

    for (int j = 0; j < 3; ++j) { // For each joint

      int jointCanId = 3 * i + j;
      quad_msgs::msg::MotorCommand &motor_command =
          leg_command.motor_commands.at(j);

      RCLCPP_INFO(
          rclcpp::get_logger("Yj01Interface"),
          "[send] leg %d, joint %d, torque %f, pos %f, vel %f, kp %f, kd %f", i,
          j, motor_command.torque_ff, motor_command.pos_setpoint,
          motor_command.vel_setpoint, motor_command.kp, motor_command.kd);

      m_link.RunMotor(jointCanId, motor_command.torque_ff,
                      motor_command.pos_setpoint, motor_command.vel_setpoint,
                      motor_command.kp, motor_command.kd);
    }
  }

  return true;
}

bool Yj01Interface::recv(sensor_msgs::msg::JointState &joint_state_msg,
                         sensor_msgs::msg::Imu &imu_msg,
                         Eigen::VectorXd &user_rx_data) {

  bool working = false;
  if (m_link.IsPortOpened(0)) {
    m_link.processCanMsgs(0);
    working = true;
  }
  if (m_link.IsPortOpened(1)) {
    m_link.processCanMsgs(1);
    working = true;
  }
  if (m_link.IsPortOpened(2)) {
    m_link.processCanMsgs(2);
    working = true;
  }
  if (m_link.IsPortOpened(3)) {
    m_link.processCanMsgs(3);
    working = true;
  }

  if (!working)
    return false;

  // Add the data corresponding to each joint
  for (int i = 0; i < joint_names_.size(); i++) {
    joint_state_msg.name[i] = joint_names_[i];
    Motor *m = m_link.GetMotor(joint_indices_[i]);
    if (m) {
      joint_state_msg.position[i] = m->m_position;
      joint_state_msg.velocity[i] = m->m_velocity;

      // Convert from current to torque using linear motor model
      joint_state_msg.effort[i] = kt_vec_[i] * m->m_torque;
    }
  }

  // process imu data

  return true;
}

void Yj01Interface::recvProc(SoulLink<RspProtocol> *p) {

  bool working = true;
  while (working) {

    working = false;
    if (p->IsPortOpened(0)) {
      p->processCanMsgs(0);
      working = true;
    }
    if (p->IsPortOpened(1)) {
      p->processCanMsgs(1);
      working = true;
    }
    if (p->IsPortOpened(2)) {
      p->processCanMsgs(2);
      working = true;
    }
    if (p->IsPortOpened(3)) {
      p->processCanMsgs(3);
      working = true;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Yj01Interface"), "recvProc finished");
}