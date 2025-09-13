#ifndef YJ01_INTERFACE_H
#define YJ01_INTERFACE_H

#include <thread>

#include <quad_msgs/msg/leg_command_array.hpp>
#include <robot_driver/hardware_interfaces/hardware_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <eigen3/Eigen/Eigen>

#include "soul_link/rsp_protocol.hpp"
#include "soul_link/soul_link.hpp"

//! Hardware interface for the Spirit40 quadruped from Ghost Robotics.
/*!
   Yj01Interface listens for joint control messages and outputs low level
   commands to the mainboard over can.
*/
class Yj01Interface : public HardwareInterface {
public:
  /**
   * @brief Constructor for Yj01Interface
   * @return Constructed object of type Yj01Interface
   */
  Yj01Interface();

  /**
   * @brief Load the hardware interface
   * @param[in] argc Argument count
   * @param[in] argv Argument vector
   */
  virtual void loadInterface(int argc, char **argv);

  /**
   * @brief Unload the hardware interface
   */
  virtual void unloadInterface();

  /**
   * @brief Send commands to the robot via the can protocol
   * @param[in] leg_command_array_msg Message containing leg commands
   * @param[in] user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
  virtual bool
  send(const quad_msgs::msg::LegCommandArray &leg_command_array_msg,
       const Eigen::VectorXd &user_tx_data);

  /**
   * @brief Recieve data from the robot via the can protocol
   * @param[out] joint_state_msg Message containing joint state information
   * @param[out] imu_msg Message containing imu information
   * @param[out] user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
  virtual bool recv(sensor_msgs::msg::JointState &joint_state_msg,
                    sensor_msgs::msg::Imu &imu_msg,
                    Eigen::VectorXd &user_rx_data);

  /// Vector of joint names
  std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};

  /// Vector denoting joint indices
  std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

  /// Vector of kt values for each joint
  std::vector<double> kt_vec_ = {0.546, 0.546, 1.092, 0.546, 0.546, 1.092,
                                 0.546, 0.546, 1.092, 0.546, 0.546, 1.092};

private:
  static void recvProc(SoulLink<RspProtocol> *p);

private:
  SoulLink<RspProtocol> m_link;
  std::thread *m_pRecvThread;
};

#endif // YJ01_INTERFACE_H
