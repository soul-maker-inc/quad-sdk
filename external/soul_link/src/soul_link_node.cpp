#include <cmath>
#include <cstdio>
#include <thread>
#include <unistd.h>

#include "soul_link/rsp_protocol.hpp"
#include "soul_link/soul_link.hpp"

void canRecv(SoulLink<RspProtocol> *link, int canPort) {
  while (link->IsPortOpened(canPort)) {
    link->processCanMsgs(canPort);
  }
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

  std::thread port0MsgHandler(canRecv, &link, 0);

  printf("querying devices...\n");
  for (int i = 1; i < 4; ++i) {
    printf("querying motor %d...\n", i);
    link.QueryMotorInfo(0, i);
  }

  // 测试关节，#1-#3
  Motor *m = link.NewMotor(0, 1);
  m->setupRobStrideRS02();
  // m->setupRobStrideRS04();

  m = link.NewMotor(0, 2);
  m->setupRobStrideRS04();

  m = link.NewMotor(0, 3);
  m->setupRobStrideRS04();

  printf("start motor reporting...\n");
  link.StartReporting(1);
  link.StartReporting(2);
  link.StartReporting(3);

  printf("enable motors...\n");
  link.EnableMotor(1);
  link.EnableMotor(2);
  link.EnableMotor(3);

  printf("set motors mode to operation mode...\n");
  link.SetMotorMode(1, RspMotorMode_Operation);
  link.SetMotorMode(2, RspMotorMode_Operation);
  link.SetMotorMode(3, RspMotorMode_Operation);

  m = link.GetMotor(1);
  m->printDebugInfo();

  float pos = 0.0f;
  float vel = 3.14f;
  float kp = 5.0f; // 100
  float kd = 0.8f; // 10
  float step = 3.14f * 0.5f;
  float torque = 0.2f;

  printf("move motor 1 to %0.3f...\n", pos);
  link.RunMotor(1, torque, pos, vel, kp, kd);
  sleep(1);
  m->printDebugInfo();

  for (int i = 0; i < 6; ++i) {
    pos += step;
    printf("move motor 1 to %0.3f...\n", pos);
    // link.RunMotor(1, torque, pos, vel, kp, kd);
    // sleep(1);
    link.RunMotor(1, torque, pos, vel, kp, kd);
    link.RunMotor(2, torque, pos, vel, kp, kd);
    link.RunMotor(3, torque, pos, vel, kp, kd);
    sleep(1);
    m->printDebugInfo();
  }

  // pos = 0.0f;
  // link.RunMotor(1, -torque, pos, -vel, kp, kd);
  // sleep(1);
  // print_motor_data(m);

  for (int i = 0; i < 6; ++i) {
    pos -= step;
    printf("move motor 1 to %0.3f...\n", pos);
    // link.RunMotor(1, -torque, pos, -vel, kp, kd);
    // sleep(1);
    link.RunMotor(1, -torque, pos, -vel, kp, kd);
    link.RunMotor(2, -torque, pos, -vel, kp, kd);
    link.RunMotor(3, -torque, pos, -vel, kp, kd);
    sleep(1);
    m->printDebugInfo();
  }

  pos = 0.0f;
  printf("move motor 1 to %0.3f...\n", pos);
  // link.RunMotor(1, torque, pos, 0.01f, kp, kd);
  // sleep(1);
  link.RunMotor(1, torque, pos, vel, kp, kd);
  link.RunMotor(2, torque, pos, vel, kp, kd);
  link.RunMotor(3, torque, pos, vel, kp, kd);
  sleep(1);
  m->printDebugInfo();

  printf("disable motors...\n");
  link.DisableMotor(1);
  link.DisableMotor(2);
  link.DisableMotor(3);

  printf("stop motors reporting...\n");
  link.StopReporting(1);
  link.StopReporting(2);
  link.StopReporting(3);

  printf("closing can port 0...\n");
  link.ClosePort(0);

  port0MsgHandler.join();

  printf("closing device 0...\n");
  link.CloseDevice();

  printf("deiniting usb...\n");
  link.DeinitUSB();
  printf("bye\n");
  return 0;
}
