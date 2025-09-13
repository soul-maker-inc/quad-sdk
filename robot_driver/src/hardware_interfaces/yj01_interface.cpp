#include "robot_driver/hardware_interfaces/yj01_interface.hpp"

Yj01Interface::Yj01Interface() : HardwareInterface(), m_pRecvThread(NULL) {}

void Yj01Interface::loadInterface(int argc, char **argv) {
  m_link.InitUSB();
  m_link.OpenDevice(0);
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
  m_pRecvThread = new std::thread(recvProc, &m_link);
}

void Yj01Interface::unloadInterface() {
  for (int i = 0; i < 4; ++i) {
    m_link.ClosePort(i);
  }
  m_link.CloseDevice();
  m_link.DeinitUSB();

  if (m_pRecvThread) {
    m_pRecvThread->join();
    delete m_pRecvThread;
    m_pRecvThread = NULL;
  }
}

bool Yj01Interface::send(
    const quad_msgs::msg::LegCommandArray &last_leg_command_array_msg,
    const Eigen::VectorXd &user_tx_data) {

  bool restart_flag = (user_tx_data[0] == 1);

  for (int i = 0; i < 4; ++i) { // For each leg
    // std::cout << "leg = " << i << std::endl;
    quad_msgs::msg::LegCommand leg_command =
        last_leg_command_array_msg.leg_commands.at(i);

    for (int j = 0; j < 3; ++j) { // For each joint
      // std::cout << "joint = " << j << std::endl;
      int jointCanId = 3 * i + j;
      quad_msgs::msg::MotorCommand &motor_command =
          leg_command.motor_commands.at(j);
      m_link.RunMotor(jointCanId, motor_command.torque_ff,
                      motor_command.pos_setpoint, motor_command.vel_setpoint,
                      motor_command.kp, motor_command.kd);
    }
  }

  // 发送关节指令

  return true;
}

bool Yj01Interface::recv(sensor_msgs::msg::JointState &joint_state_msg,
                         sensor_msgs::msg::Imu &imu_msg,
                         Eigen::VectorXd &user_rx_data) {

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
}