#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"
#include <math.h>

// PI
#define PI 3.1415926535897932384626433832795f

//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
