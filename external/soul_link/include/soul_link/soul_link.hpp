#pragma once

#include <cstddef>
#include <map>

#include "iTekCANFD.h"
#include "motor.hpp"

enum MotorProtocol {
  MotorProtocol_Private = 0,
  MotorProtocol_CanOpen = 1,
  MotorProtocol_MIT = 2,
};

enum RspMotorMode {
  RspMotorMode_Operation = 0,
  RspMotorMode_PositionPP = 1,
  RspMotorMode_Velocity = 2,
  RspMotorMode_Current = 3,
  RspMotorMode_PositionCSP = 5,
};

enum MitMotorMode {
  MitMotorMode_MIT = 0,
  MitMotorMode_Position = 1,
  MitMotorMode_Velocity = 2,
};

/**
 * CAN盒
 */
template <typename t_protocol> class SoulLink : public t_protocol {
public:
  SoulLink();
  ~SoulLink();

  static bool InitUSB();
  static void DeinitUSB();

  bool OpenDevice(int devIndex = 0);
  void CloseDevice();
  void PrintDeviceInfo();

  bool OpenPort(int portIndex);
  void ClosePort(int portIndex);

  Motor *NewMotor(int canPortIndex, int motorId);

  inline Motor *GetMotor(int motorId) {
    auto it = m_motors.find(motorId);
    return it == m_motors.end() ? NULL : &it->second;
  }

public:
  uint32_t sendCanExtFrame(int canPortIndex, canid_t frameid,
                           uint8_t *data = NULL, int dataSize = 8);
  uint32_t sendCanStdFrame(int canPortIndex, canid_t frameid,
                           uint8_t *data = NULL, int dataSize = 8);

  inline void processCanMsgs(int canPortIndex) {
    recvCanMsgs(m_canPorts[canPortIndex]);
  }

protected:
  void recvCanMsgs(CHANNEL_HANDLE canhandle);

private:
  std::map<int /* motorId */, Motor> m_motors;
  DEVICE_HANDLE m_dev;
  CHANNEL_HANDLE m_canPorts[4]; // current device only support 4 ports
  uint8_t m_hostId;
  bool m_mitProtocol;

  friend class RspProtocol;
  friend class MitProtocol;
};

#include "soul_link/soul_link_impl.hpp"