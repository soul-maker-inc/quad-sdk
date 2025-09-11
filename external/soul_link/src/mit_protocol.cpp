#include "soul_link/mit_protocol.hpp"

MitProtocol::MitProtocol(SoulLink<MitProtocol> *link) : m_link(link) {}

MitProtocol::~MitProtocol() {}

void MitProtocol::QueryMotorInfo(int canPort, int motorCanId) {}

void MitProtocol::SetMotorProtocol(int motorId, MotorProtocol protocol) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  data[6] = protocol;
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::SetMotorId(int motorId, int newMotorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA};
  data[6] = newMotorId;
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::SetMotorPosZero(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::EnableMotor(int motorId) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::DisableMotor(int motorId) {

  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::SetMotorMode(int motorId, MitMotorMode mode) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  data[6] = mode;
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::RunMotor(int motorId, float torque, float angle,
                           float angular_speed, float kp, float kd) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8];
  data[0] = float_to_uint(angle, m->m_posRange[0], m->m_posRange[1], 16) >> 8;
  data[1] = float_to_uint(angle, m->m_posRange[0], m->m_posRange[1], 16) & 0xff;
  data[2] =
      float_to_uint(angular_speed, m->m_velRange[0], m->m_velRange[1], 12) >> 4;
  data[3] =
      (float_to_uint(angular_speed, m->m_velRange[0], m->m_velRange[1], 12) &
       0x0f) +
      (float_to_uint(kp, m->m_kpRange[0], m->m_kpRange[1], 12) >> 8);
  data[4] = float_to_uint(kp, m->m_kpRange[0], m->m_kpRange[1], 12) & 0xff;
  data[5] = float_to_uint(kd, m->m_kdRange[0], m->m_kdRange[1], 12) >> 4;
  data[6] =
      (float_to_uint(kd, m->m_kdRange[0], m->m_kdRange[1], 12) & 0x0f) +
      (float_to_uint(torque, m->m_torqueRange[0], m->m_torqueRange[1], 12) >>
       8);
  data[7] =
      float_to_uint(torque, m->m_torqueRange[0], m->m_torqueRange[1], 12) &
      0xff;
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::SetMotorPosition(int motorId, float pos,
                                   float angular_speed) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8];
  memcpy(data, &pos, 4);
  memcpy(data + 4, &angular_speed, 4);
  m_link->sendCanStdFrame(m->m_canPort, 0x0100 | m->m_canId, data, 8);
}

void MitProtocol::SetMotorVelocity(int motorId, float angular_speed,
                                   float limit) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8];
  memcpy(data, &angular_speed, 4);
  memcpy(data + 4, &limit, 4);
  m_link->sendCanStdFrame(m->m_canPort, 0x0200 | m->m_canId, data, 8);
}

void MitProtocol::ClearMotorError(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::GetMotorError(int motorId) {
  Motor *m = m_link->GetMotor(motorId);
  if (m == NULL)
    return;

  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFB};
  m_link->sendCanStdFrame(m->m_canPort, m->m_canId, data, 8);
}

void MitProtocol::handleCanMsg(const iTek_CANFD_Receive_Data &recvdata) {

  int frameid = recvdata.frame.can_id; // 标准帧，只有11bit
  if (frameid == m_link->m_hostId) {
    int motorId = recvdata.frame.data[0];
    Motor *m = m_link->GetMotor(motorId);
    if (m) {
      if (recvdata.frame.data[5] != 0 || recvdata.frame.data[6] != 0 ||
          recvdata.frame.data[7] != 0) {
        // 应答指令1
        m->m_position = uint16_to_float(recvdata.frame.data[2] +
                                            (recvdata.frame.data[1] << 8),
                                        m->m_posRange[0], m->m_posRange[1], 16);
        m->m_velocity = uint16_to_float((recvdata.frame.data[3] << 8) +
                                            (recvdata.frame.data[4] >> 4),
                                        m->m_velRange[0], m->m_velRange[1], 12);
        m->m_torque = uint16_to_float(
            ((recvdata.frame.data[4] & 0x0f) << 4) + recvdata.frame.data[5],
            m->m_torqueRange[0], m->m_torqueRange[1], 12);
        m->m_temperature =
            (float)(recvdata.frame.data[7] + (recvdata.frame.data[6] << 8));
      } else {
        // 故障应答
        // Byte1~4: fault值(非0:有故障，0：正常）
        //   bit14:堵转i方t过载故障
        //   bit7:编码器未标定
        //   bit3:过压故障
        //   bit2:欠压故障
        //   bit1:驱动芯片故障
        //   bit0:电机过温故障，默认145度
        m->m_error = recvdata.frame.data[1] + (recvdata.frame.data[2] << 8) +
                     (recvdata.frame.data[3] << 16) +
                     (recvdata.frame.data[4] << 24);
      }
    }
  } else {
    // 应答指令2
    int motorId = frameid;
    Motor *m = m_link->GetMotor(motorId);
    if (m) {
      memcpy(m->m_mcuId, recvdata.frame.data, 8);
    }
  }
}
