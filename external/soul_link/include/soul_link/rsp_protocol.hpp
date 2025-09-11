#pragma once
#include "soul_link/motor.hpp"
#include "soul_link/soul_link.hpp"

class RspProtocol {
public:
  RspProtocol(SoulLink<RspProtocol> *link);
  ~RspProtocol();

  // RobStride私有协议
  void QueryMotorInfo(int canPort, int motorCanId);
  void SetMotorProtocol(int motorId, MotorProtocol protocol);
  void SetMotorId(int motorId, int newMotorId);
  void SetMotorPosZero(int motorId);
  void EnableMotor(int motorId);
  void DisableMotor(int motorId);
  void SetMotorMode(int motorId, RspMotorMode mode);
  void RunMotor(int motorId, float torque, float angle, float angular_speed,
                float kp, float kd);
  void SetMotorPosition(int motorId, float pos, float _dummy);
  void SetMotorVelocity(int motorId, float angular_speed, float limit);
  void ClearMotorError(int motorId);
  void handleCanMsg(const iTek_CANFD_Receive_Data &recvdata);

private:
  SoulLink<RspProtocol> *m_link;
};