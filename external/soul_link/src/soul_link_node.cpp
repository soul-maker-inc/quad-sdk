#include <cmath>
#include <cstdio>
#include <unistd.h>

#include "soul_link/rsp_protocol.hpp"
#include "soul_link/soul_link.hpp"

void print_motor_data(Motor *m) {
  printf("motor 1 pos: %0.3f\n", m->m_position);
  printf("motor 1 T: %0.3f\n", m->m_torque);
  printf("motor 1 vel: %0.3f\n", m->m_velocity);
  printf("motor 1 temp: %0.3f\n", m->m_temperature);
  printf("motor 1 warn: %X\n", m->m_warnning);
  printf("motor 1 error: %X\n-------------------\n", m->m_error);
}

int main(int argc, char **argv) {

  SoulLink<RspProtocol> link;

  printf("initing usb...");
  if (!link.InitUSB()) {
    printf("failed");
  }
  int dev = 0;
  printf("\nopenning device %d...\n", dev);
  if (!link.OpenDevice(dev)) {
    printf("failed\n");
  }
  link.PrintDeviceInfo();
  printf("openning can port 0...\n");
  if (!link.OpenPort(0)) {
    printf("failed\n");
  }

  printf("querying devices...\n");
  for (int i = 1; i < 4; ++i) {
    printf("querying motor %d...\n", i);
    link.QueryMotorInfo(0, i);
    link.processCanMsgs(0);
  }
  link.processCanMsgs(0);

  // 测试关节，#1-#3
  Motor *m = link.NewMotor(0, 1);
  m->setupRobStrideRS02();

  m = link.NewMotor(0, 2);
  m->setupRobStrideRS04();

  m = link.NewMotor(0, 3);
  m->setupRobStrideRS04();

  printf("enable motors...\n");
  link.EnableMotor(1);
  link.EnableMotor(2);
  link.EnableMotor(3);

  printf("set motors mode to positionPP...\n");
  link.SetMotorMode(1, RspMotorMode_Operation);
  link.SetMotorMode(2, RspMotorMode_Operation);
  link.SetMotorMode(3, RspMotorMode_Operation);

  link.processCanMsgs(0);

  m = link.GetMotor(1);
  print_motor_data(m);

  float pos = 0.0f;
  float vel = 0.1f;
  float kp = 2.0f;
  float kd = 0.3f;
  float step = 0.523f;
  float torque = 1.0f;

  link.RunMotor(1, torque, pos, vel, kp, kd);
  sleep(1);
  link.processCanMsgs(0);
  print_motor_data(m);

  for (int i = 0; i < 12; ++i) {
    pos += step;
    printf("move motor 1 to %0.3f...\n", pos);
    // link.RunMotor(1, torque, pos, vel, kp, kd);
    // sleep(1);
    // link.processCanMsgs(0);
    link.RunMotor(1, torque, pos, vel, kp, kd);
    sleep(1);
    link.processCanMsgs(0);
    print_motor_data(m);
  }

  pos = 0.0f;
  link.RunMotor(1, -torque, pos, -vel, kp, kd);
  sleep(1);
  link.processCanMsgs(0);
  print_motor_data(m);

  for (int i = 0; i < 12; ++i) {
    pos -= step;
    printf("move motor 1 to %0.3f...\n", pos);
    // link.RunMotor(1, -torque, pos, -vel, kp, kd);
    // sleep(1);
    // link.processCanMsgs(0);
    link.RunMotor(1, -torque, pos, -vel, kp, kd);
    sleep(1);
    link.processCanMsgs(0);
    print_motor_data(m);
  }

  pos = 0.0f;
  printf("move motor 1 to %0.3f...\n", pos);
  // link.RunMotor(1, torque, pos, 0.01f, kp, kd);
  // sleep(1);
  // link.processCanMsgs(0);
  link.RunMotor(1, torque, pos, vel, kp, kd);
  sleep(1);
  link.processCanMsgs(0);
  print_motor_data(m);

  printf("disable motors...\n");
  link.DisableMotor(1);
  link.DisableMotor(2);
  link.DisableMotor(3);

  printf("closing can port 0...\n");
  link.ClosePort(0);

  printf("closing device 0...\n");
  link.CloseDevice();

  printf("deiniting usb...\n");
  link.DeinitUSB();
  printf("bye\n");
  return 0;
}
