#include "soul_link/utils.h"
#include <cstdio>
#include <cstring>

//-------------------------------------------------------------------------------

template <typename t_protocol> bool SoulLink<t_protocol>::InitUSB() {
  return iTek_UsbInit();
}

template <typename t_protocol> void SoulLink<t_protocol>::DeinitUSB() {
  iTek_UsbExit();
}

template <typename t_protocol>
SoulLink<t_protocol>::SoulLink()
    : t_protocol(this), m_dev(NULL), m_hostId(0xfd), m_mitProtocol(false) {
  memset(&m_canPorts, 0, sizeof(CHANNEL_HANDLE) * 4);
}

template <typename t_protocol> SoulLink<t_protocol>::~SoulLink() {}

template <typename t_protocol>
bool SoulLink<t_protocol>::OpenDevice(int devIndex) {

  if (m_dev != NULL)
    return false;

  DEVICE_HANDLE devhandle = iTek_OpenDevice(2, devIndex, 0);

  if (NULL == devhandle) {
    return false;
  }

  m_dev = devhandle;

  return true;
}

template <typename t_protocol> void SoulLink<t_protocol>::CloseDevice() {
  if (m_dev) {
    iTek_CloseDevice(m_dev);
    m_dev = NULL;
  }
}

template <typename t_protocol> void SoulLink<t_protocol>::PrintDeviceInfo() {

  if (m_dev == NULL)
    return;

  iTek_CANFD_DEVICE_INFO devinfo;
  iTek_GetDeviceInfo(m_dev, &devinfo);
  printf("硬件版本:V%d.%d.%d\n", devinfo.hw_Version[0], devinfo.hw_Version[1],
         devinfo.hw_Version[2]);
  printf("固件版本:V%d.%d.%d\n", devinfo.fw_Version[0], devinfo.fw_Version[1],
         devinfo.fw_Version[2]);
  printf("通道数量:%d\n", devinfo.can_Num);
  printf("出厂序列号:%s\n", devinfo.str_Serial_Num);
  printf("设备名称描述字符:%s\n", devinfo.str_hw_Type);
}

template <typename t_protocol>
bool SoulLink<t_protocol>::OpenPort(int portIndex) {

  if (portIndex < 0 || portIndex > 3) {
    printf("invalid port index %d\n", portIndex);
    return false;
  }
  if (m_dev == NULL) {
    printf("invalid dev handle\n");
    return false;
  }
  if (m_canPorts[portIndex] != NULL) {
    printf("port %d already opened\n", portIndex);
    return false;
  }

  iTek_CANFD_CHANNEL_INIT_CONFIG can_cfg;

  memset(&can_cfg, 0, sizeof(can_cfg));

  can_cfg.can_type = 0;
  can_cfg.CANFDStandard = 0;
  can_cfg.CANFDSpeedup = 0;
  can_cfg.workMode = 0;
  can_cfg.abit_timing = Arbitrat_Rate1000K;
  can_cfg.dbit_timing = Data_Rate1M;
  can_cfg.Standard.num = 1;
  can_cfg.Standard.filterDataStandard[0].ID1 = 0x00;
  can_cfg.Standard.filterDataStandard[0].ID2 = 0x7ff;
  can_cfg.Standard.filterDataStandard[0].frameType = 0;
  can_cfg.Standard.filterDataStandard[0].filterType = 0;
  can_cfg.Extend.num = 1;
  can_cfg.Extend.filterDataExtend[0].ID1 = 0x00;
  can_cfg.Extend.filterDataExtend[0].ID2 = 0x1FFFFFFF;
  can_cfg.Extend.filterDataExtend[0].frameType = 1;
  can_cfg.Extend.filterDataExtend[0].filterType = 0;

  CHANNEL_HANDLE candev = iTek_InitCan(m_dev, portIndex, &can_cfg);

  if (candev == NULL) {
    printf("cannot init can port %d\n", portIndex);
    return false;
  }

  if (!iTek_StartCAN(candev)) {
    iTek_RestCAN(candev);
    printf("cannot start can port %d\n", portIndex);
    return false;
  }

  m_canPorts[portIndex] = candev;

  return true;
}

template <typename t_protocol>
void SoulLink<t_protocol>::ClosePort(int portIndex) {

  if (portIndex < 0 || portIndex > 3)
    return;
  if (m_dev)
    return;
  if (m_canPorts[portIndex] == NULL)
    return;

  CHANNEL_HANDLE candev = m_canPorts[portIndex];
  iTek_ClearBuffer(candev);
  iTek_RestCAN(candev);
  m_canPorts[portIndex] = NULL;
}

template <typename t_protocol>
Motor *SoulLink<t_protocol>::NewMotor(int canPortIndex, int motorId) {
  auto it = m_motors.find(motorId);
  if (it != m_motors.end())
    return &it->second;

  m_motors[motorId] = std::move(Motor(motorId, canPortIndex));
  return &m_motors[motorId];
}

template <typename t_protocol>
void SoulLink<t_protocol>::recvCanMsgs(CHANNEL_HANDLE canhandle) {

  iTek_CANFD_Receive_Data recvdata[100];
  int recvnum = iTek_Receive(canhandle, recvdata, 100, 2);

  for (int j = 0; j < recvnum; j++) {
    printf("[%d]recvtime:[%lx] canid:[%x] cantype:[%d] datalen:[%d] data[", j,
           recvdata[j].timestamp, recvdata[j].frame.can_id & 0x1fffffff,
           recvdata[j].frame.cantype, recvdata[j].frame.len);

    for (int i = 0; i < recvdata[j].frame.len; i++) {
      printf("%02x ", recvdata[j].frame.data[i]);
    }

    printf("]\n");

    ((t_protocol *)this)->handleCanMsg(recvdata[j]);
  }
}

template <typename t_protocol>
uint32_t SoulLink<t_protocol>::sendCanExtFrame(int canPortIndex,
                                               canid_t frameid, uint8_t *data,
                                               int dataSize) {

  printf("sending can ext frame, port = %d, frame id = %X\n", canPortIndex,
         frameid);
  CHANNEL_HANDLE canhandle = m_canPorts[canPortIndex];

  iTek_CANFD_Transmit_Data transmit_data;
  memset(&transmit_data, 0, sizeof(iTek_CANFD_Transmit_Data));

  transmit_data.send_type = 0;
  transmit_data.frame.can_id = (0x40 << 24) | frameid;

  transmit_data.frame.cantype = 0; /*0:CAN 2:CANFD 3:CANFD加速*/
  transmit_data.frame.len = dataSize;

  if (data == NULL)
    memset(transmit_data.frame.data, 0, dataSize);
  else
    memcpy(transmit_data.frame.data, data, dataSize);

  return iTek_Transmit(canhandle, &transmit_data, 1);
}

template <typename t_protocol>
uint32_t SoulLink<t_protocol>::sendCanStdFrame(int canPortIndex,
                                               canid_t frameid, uint8_t *data,
                                               int dataSize) {

  printf("sending can std frame, port = %d, frame id = %X\n", canPortIndex,
         frameid);
  CHANNEL_HANDLE canhandle = m_canPorts[canPortIndex];

  iTek_CANFD_Transmit_Data transmit_data;
  memset(&transmit_data, 0, sizeof(iTek_CANFD_Transmit_Data));

  transmit_data.send_type = 0;
  transmit_data.frame.can_id = frameid & 0x07ff; // 11bit

  transmit_data.frame.cantype = 0; /*0:CAN 2:CANFD 3:CANFD加速*/
  transmit_data.frame.len = dataSize;

  if (data == NULL)
    memset(transmit_data.frame.data, 0, dataSize);
  else
    memcpy(transmit_data.frame.data, data, dataSize);

  return iTek_Transmit(canhandle, &transmit_data, 1);
}
