/**
  ******************************************************************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date          Author          Modification
  *  V1.2.0     2023-8-18     Shidong Wu      增加离散PID计算函数
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
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
/**
  * @brief          pid初始化
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION或PID_DELTA
  * @param[in]      kp: Kp
  * @param[in]      ki: Ki
  * @param[in]      kd: Kd
  * @param[in]      max_out: 最大输出
  * @param[in]      max_iout: 最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);



/**
  * @brief          pid计算（离散型）
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @param[in]      dt: 采样时间
  * @retval         pid输出
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
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
