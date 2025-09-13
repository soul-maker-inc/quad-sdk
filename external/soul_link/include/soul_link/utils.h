#pragma once

#include <stdint.h>

/*仲裁域定义*/
#define Arbitrat_Rate5K 0x01F31302
#define Arbitrat_Rate10K 0x00F91302
#define Arbitrat_Rate20K 0x007C1302
#define Arbitrat_Rate40K 0x00630A02
#define Arbitrat_Rate50K 0x00311302
#define Arbitrat_Rate80K 0x001D1204
#define Arbitrat_Rate100K 0x00181302
#define Arbitrat_Rate125K 0x00131302
#define Arbitrat_Rate200K 0x00130A02
#define Arbitrat_Rate250K 0x00091302
#define Arbitrat_Rate400K 0x00090A02
#define Arbitrat_Rate500K 0x00070A02
#define Arbitrat_Rate800K 0x00021006
#define Arbitrat_Rate1000K 0x00040702

/*数据域定义*/
#define Data_Rate100K 0x001D0E30
#define Data_Rate125K 0x001F0A20
#define Data_Rate200K 0x00130A20
#define Data_Rate250K 0x000F0A20
#define Data_Rate400K 0x00090A20
#define Data_Rate500K 0x00090A20
#define Data_Rate800K 0x00040A20
#define Data_Rate1M 0x00040720
#define Data_Rate2M 0x00010A20
#define Data_Rate3M 0x00000D40
#define Data_Rate4M 0x00000A20
#define Data_Rate5M 0x00000720
#define Data_Rate6M 0x00010200

float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
float Byte_to_float(uint8_t *bytedata);
