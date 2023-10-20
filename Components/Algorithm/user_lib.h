#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"
#include <math.h>

// PI
#define PI 3.1415926535897932384626433832795f

//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
