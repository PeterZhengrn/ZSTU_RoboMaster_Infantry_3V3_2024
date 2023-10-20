/**
  ******************************************************************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date          Author          Modification
  *  V1.2.0     2023-8-18     Shidong Wu      ������ɢPID���㺯��
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;
/**
  * @brief          pid��ʼ��
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION��PID_DELTA
  * @param[in]      kp: Kp
  * @param[in]      ki: Ki
  * @param[in]      kd: Kd
  * @param[in]      max_out: ������
  * @param[in]      max_iout: ���������
  * @retval         none
*/
void PID_init(pid_type_def *pid, uint8_t mode, fp32 kp,fp32 ki,fp32 kd, fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);



/**
  * @brief          pid���㣨��ɢ�ͣ�
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @param[in]      dt: ����ʱ��
  * @retval         pid���
  */
/**
 * @brief          pid calculate (discrete)
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @param[in]      dt: sample time
 * @retval         pid out
 */
extern fp32 Discrete_PID_calc(pid_type_def *pid, fp32 ref, fp32 set,fp32 dt);



/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
