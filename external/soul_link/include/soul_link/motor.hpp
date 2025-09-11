#pragma once

#include <stdint.h>

class Motor {
public:
  Motor();
  Motor(int id, int canPort);
  Motor(const Motor &m);
  ~Motor();

  void setupRobStrideRS02();
  void setupRobStrideRS04();

protected:
  void initMotorParams();

public:
  // 电机状态
  volatile float m_position;    // 位置（弧度）
  volatile float m_velocity;    // 角速度
  volatile float m_torque;      // 扭矩
  volatile float m_temperature; // 温度

  // bit14:堵转i方t过载故障
  // bit7:编码器未标定
  // bit3:过压故障
  // bit2:欠压故障
  // bit1:驱动芯片故障
  // bit0:电机过温故障，默认135度
  volatile uint32_t m_error;

  // Byte4~7: warning值
  // bit0：电机过温预警，默认125度
  volatile uint32_t m_warnning;

private:
  // 关节CAN ID
  int m_canId;
  int m_canPort;
  uint8_t m_mcuId[8];

  // 电机参数
  float m_posRange[2];
  float m_velRange[2];
  float m_kpRange[2];
  float m_kdRange[2];
  float m_torqueRange[2];

  friend class MitProtocol;
  friend class RspProtocol;
};
