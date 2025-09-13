#include "soul_link/rsp_protocol.hpp"
#include "soul_link/soul_link.hpp"

RspProtocol::RspProtocol(SoulLink<RspProtocol> *link) : m_link(link) {}

RspProtocol::~RspProtocol() {}

// RobStride私有协议
void RspProtocol::QueryMotorInfo(int canPort, int motorCanId) {
  m_link->sendCanExtFrame(canPort,
                          (0x00 << 24) + (m_link->m_hostId << 8) + motorCanId);
}

void RspProtocol::SetMotorProtocol(int motorId, MotorProtocol protocol) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};
  data[6] = protocol;

  m_link->sendCanExtFrame(m->m_canPort,
                          (0x19 << 24) + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::SetMotorId(int motorId, int newMotorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  m_link->sendCanExtFrame(m->m_canPort, (0x07 << 24) + (newMotorId << 16) +
                                            (m_link->m_hostId << 8) +
                                            m->m_canId);
}

void RspProtocol::SetMotorPosZero(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
  m_link->sendCanExtFrame(m->m_canPort,
                          (0x06 << 24) + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::EnableMotor(int motorId) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  m_link->sendCanExtFrame(m->m_canPort,
                          (0x03 << 24) + (m_link->m_hostId << 8) + m->m_canId);
}

void RspProtocol::DisableMotor(int motorId) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  m_link->sendCanExtFrame(m->m_canPort,
                          (0x04 << 24) + (m_link->m_hostId << 8) + m->m_canId);
}

void RspProtocol::SetMotorMode(int motorId, RspMotorMode mode) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;
  uint8_t data[8] = {0x05, 0x70, 0, 0, 0x01, 0x00, 0x00, 0x00};
  data[4] = mode;

  m_link->sendCanExtFrame(m->m_canPort,
                          (0x12 << 24) + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::RunMotor(int motorId, float torque, float angle,
                           float angular_speed, float kp, float kd) {
  printf("-- run_motor [%d]: T%.3f, to %.3f rad, sp %.3f, kp = %.3f, kd = "
         "%.3f\n",
         motorId, torque, angle, angular_speed, kp, kd);

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0};
  uint32_t t;

  t = float_to_uint(angle, m->m_posRange[0], m->m_posRange[1], 16);
  data[0] = t >> 8;
  data[1] = t & 0xff;

  t = float_to_uint(angular_speed, m->m_velRange[0], m->m_velRange[1], 16);
  data[2] = t >> 8;
  data[3] = t & 0xff;

  t = float_to_uint(kp, m->m_kpRange[0], m->m_kpRange[1], 16);
  data[4] = t >> 8;
  data[5] = t & 0xff;

  t = float_to_uint(kd, m->m_kdRange[0], m->m_kdRange[1], 16);
  data[6] = t >> 8;
  data[7] = t & 0xff;

  t = float_to_uint(torque, m->m_torqueRange[0], m->m_torqueRange[1], 16);
  uint32_t itorque = t; // (t >> 8) + ((t & 0xff) << 8); // 交换高低字节顺序

  m_link->sendCanExtFrame(m->m_canPort,
                          (0x01 << 24)                    // b28-24: type
                              + ((itorque & 0xffff) << 8) // b23-8: torque
                              + m->m_canId,
                          data, 8);
}

void RspProtocol::SetMotorPosition(int motorId, float pos, float _dummy) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0x16, 0x70, 0, 0, 0x00, 0x00, 0x00, 0x00};
  memcpy(data + 4, &pos, 4);
  m_link->sendCanExtFrame(m->m_canPort,
                          (0x12 << 24) // b28-24: type
                              + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::SetMotorVelocity(int motorId, float angular_speed,
                                   float limit) {}

void RspProtocol::ClearMotorError(int motorId) {}

void RspProtocol::StartReporting(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00};
  m_link->sendCanExtFrame(m->m_canPort,
                          (0x18 << 24) // b28-24: type
                              + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::StopReporting(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};
  m_link->sendCanExtFrame(m->m_canPort,
                          (0x18 << 24) // b28-24: type
                              + (m_link->m_hostId << 8) + m->m_canId,
                          data, 8);
}

void RspProtocol::handleCanMsg(const iTek_CANFD_Receive_Data &recvdata) {

  int frameid = recvdata.frame.can_id & 0x1fffffff;
  switch (frameid >> 24) {
  case 0x00: {
    // 设备ID帧
    int motorId = (frameid & 0x0000ff00) >> 8; // Bit8~Bit15:当前电机CAN ID
    if ((frameid & 0xff) == 0xfe) {
      Motor *m = m_link->GetMotor(motorId);
      if (m)
        memcpy(m->m_mcuId, recvdata.frame.data, 8);
      // printf(
      //     "recv motor %d chip id:
      //     [%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X]\n", motorId,
      //     recvdata.frame.data[0], recvdata.frame.data[1],
      //     recvdata.frame.data[2], recvdata.frame.data[3],
      //     recvdata.frame.data[4], recvdata.frame.data[5],
      //     recvdata.frame.data[6], recvdata.frame.data[7]);
    }
    break;
  }
  case 0x02:
  case 0x18: {
    // 电机反馈数据
    int motorId = (frameid & 0x0000ff00) >> 8; // Bit8~Bit15:当前电机CAN ID
    int errorCode =
        (frameid & 0x003f0000) >> 16;           // bit21~16:故障信息（0无 1有）
    bool isCalibrateErr = (errorCode & 0x0020); // bit21: 未标定
    bool overloadErr = (errorCode & 0x0010);    // bit20: 堵转过载故障
    bool encoderErr = (errorCode & 0x0008);     // bit19: 磁编码故障
    bool overheatErr = (errorCode & 0x0004);    // bit18: 过温
    bool overCurrentErr = (errorCode & 0x0002); // bit17: 过流
    bool lowVoltageErr = (errorCode & 0x0001);  // bit16: 欠压故障
    int mode = (frameid & 0x00c00000) >> 22;    // bit22~23: 模式状态
                                                //  0 : Reset 模式[复位]
                                                //  1 : Cali 模式[标定]
                                                //  2 : Motor模式[运行]
    // --------数据解析-------------------
    Motor *m = m_link->GetMotor(motorId);
    if (m) {
      if (isCalibrateErr)
        m->m_error |= 0x80;
      if (overloadErr)
        m->m_error |= 0x4000;
      if (lowVoltageErr)
        m->m_error |= 0x04;
      if (overheatErr)
        m->m_error |= 0x01;
      m->m_position = uint16_to_float(recvdata.frame.data[1] +
                                          (recvdata.frame.data[0] << 8),
                                      m->m_posRange[0], m->m_posRange[1], 16);
      m->m_velocity = uint16_to_float(recvdata.frame.data[3] +
                                          (recvdata.frame.data[2] << 8),
                                      m->m_velRange[0], m->m_velRange[1], 16);
      m->m_torque = uint16_to_float(
          recvdata.frame.data[5] + (recvdata.frame.data[4] << 8),
          m->m_torqueRange[0], m->m_torqueRange[1], 16);
      m->m_temperature =
          (float)(recvdata.frame.data[7] + (recvdata.frame.data[6] << 8)) /
          10.0f;

      // m->printDebugInfo();
    }
    break;
  }
  case 0x15: {
    // 故障反馈帧
    int motorId = (frameid & 0x0000ff00) >> 8; // Bit8~Bit15:当前电机CAN ID
    Motor *m = m_link->GetMotor(motorId);
    if (m) {
      m->m_error = recvdata.frame.data[0] + (recvdata.frame.data[1] << 8) +
                   (recvdata.frame.data[2] << 16) +
                   (recvdata.frame.data[3] << 24);
      m->m_warnning = recvdata.frame.data[4] + (recvdata.frame.data[5] << 8) +
                      (recvdata.frame.data[6] << 16) +
                      (recvdata.frame.data[7] << 24);
    }
    break;
  }
  }
}
