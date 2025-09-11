#pragma once
#include "soul_link/motor.hpp"
#include "soul_link/soul_link.hpp"

class MitProtocol {

public:
  MitProtocol(SoulLink<MitProtocol> *link);
  ~MitProtocol();

  void QueryMotorInfo(int canPort, int motorCanId);
  void SetMotorProtocol(int motorId, MotorProtocol protocol);
  void SetMotorId(int motorId, int newMotorId);
  void SetMotorPosZero(int motorId);
  void EnableMotor(int motorId);
  void DisableMotor(int motorId);
  void SetMotorMode(int motorId, MitMotorMode mode);
  void RunMotor(int motorId, float torque, float angle, float angular_speed,
                float kp, float kd);
  void SetMotorPosition(int motorId, float pos, float _dummy);
  void SetMotorVelocity(int motorId, float angular_speed, float limit);
  void ClearMotorError(int motorId);
  void GetMotorError(int motorId);
  void handleCanMsg(const iTek_CANFD_Receive_Data &recvdata);

private:
  SoulLink<MitProtocol> *m_link;
};