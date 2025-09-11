#include "soul_link/motor.hpp"
#include "soul_link/soul_link.hpp"

Motor::Motor() : m_canId(0), m_canPort(0), m_error(0), m_warnning(0) {
  initMotorParams();
}

Motor::Motor(int id, int canPort)
    : m_canId(id), m_canPort(canPort), m_error(0), m_warnning(0) {
  initMotorParams();
}

Motor::Motor(const Motor &m)
    : m_canId(m.m_canId), m_canPort(m.m_canPort), m_error(0), m_warnning(0) {
  memcpy(m_mcuId, m.m_mcuId, 8);
  m_posRange[0] = m.m_posRange[0];
  m_posRange[1] = m.m_posRange[1];
  m_velRange[0] = m.m_velRange[0];
  m_velRange[1] = m.m_velRange[1];
  m_kpRange[0] = m.m_kpRange[0];
  m_kpRange[1] = m.m_kpRange[1];
  m_kdRange[0] = m.m_kdRange[0];
  m_kdRange[1] = m.m_kdRange[1];
  m_torqueRange[0] = m.m_torqueRange[0];
  m_torqueRange[1] = m.m_torqueRange[1];
}

Motor::~Motor() {}

void Motor::initMotorParams() {
  memset(m_mcuId, 0, 8);
  m_posRange[0] = 0;
  m_posRange[1] = 0;
  m_velRange[0] = 0;
  m_velRange[1] = 0;
  m_kpRange[0] = 0;
  m_kpRange[1] = 0;
  m_kdRange[0] = 0;
  m_kdRange[1] = 0;
  m_torqueRange[0] = 0;
  m_torqueRange[1] = 0;
}

void Motor::setupRobStrideRS02() {

  m_posRange[0] = -12.5f;
  m_posRange[1] = 12.5f;
  m_velRange[0] = -44.0f;
  m_velRange[1] = 44.0f;
  m_kpRange[0] = 0.0f;
  m_kpRange[1] = 500.0f;
  m_kdRange[0] = 0.0f;
  m_kdRange[1] = 5.0f;
  m_torqueRange[0] = -17.0f;
  m_torqueRange[1] = 17.0f;
}

void Motor::setupRobStrideRS04() {
  m_posRange[0] = -12.57f;
  m_posRange[1] = 12.57f;
  m_velRange[0] = -15.0f;
  m_velRange[1] = 15.0f;
  m_kpRange[0] = 0.0f;
  m_kpRange[1] = 5000.0f;
  m_kdRange[0] = 0.0f;
  m_kdRange[1] = 100.0f;
  m_torqueRange[0] = -120.0f;
  m_torqueRange[1] = 120.0f;
}
