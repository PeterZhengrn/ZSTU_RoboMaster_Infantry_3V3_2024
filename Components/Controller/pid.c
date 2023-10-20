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

#include "pid.h"
#include "main.h"
/***********************template PID����ģ��***************************
#define kp 0.0f  //kp
#define ki 0.0f  //ki
#define kd 0.0f  //kd
#define dt 0.001f //����ʱ��
pid_type_def pid_x; //pid�ṹ��
fp32 PID_x[3] = {kp, ki, kd}; //pid����
int main(void)
{
    PID_init(&pid_x, PID_POSITION, PID_x, 1000, 1000);//pid��ʼ��
    while(1)
    {
        PID_calc(&pid_x, ref, set);//pid����
        Discrete_PID_calc(&pid_x, ref, set, dt);//��ɢpid����
        osDelay(dt*1000);//��ʱ
    }
}
*********************************************************************/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


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
void PID_init(pid_type_def *pid, uint8_t mode, fp32 kp,fp32 ki,fp32 kd, fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid���㣨�����ͣ�
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}



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

fp32 Discrete_PID_calc(pid_type_def *pid, fp32 ref, fp32 set,fp32 dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)//λ��ʽPID
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*dt;//��ɢ������
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0]/dt;//��ɢ΢����
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;//λ��ʽPID���
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)//����ʽPID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0]*dt;//��ɢ������
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0]/dt;//��ɢ΢����
        pid->out += pid->Pout + pid->Iout + pid->Dout;//����ʽPID���
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
