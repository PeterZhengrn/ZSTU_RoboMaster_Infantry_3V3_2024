#ifndef DEFAULT_TASK_H__
#define DEFAULT_TASK_H__
#ifdef __cplusplus
extern "C"
{
#endif
/* USER CODE BEGIN Includes Form C Files*/
#include "main.h"
#include "struct_typedef.h"
#include "remote_control.h"
// typedef enum
// {
//     //底盘类型
//     CHASSIS_MECANUM = 0, //麦克纳姆轮
//     CHASSIS_OMNI,       //全向轮
//     CHASSIS_STEER,//舵轮
// }chassis_type_e;//底盘类型

// typedef enum
// {
//     //云台类型
//     GIMBAL_SINGLE=0,     //单云台
//     GIMBAL_DOUBLE, //双云台
// }gimbal_type_e;//云台类型
#define ROBOT_MODE_CHANNEL 0 //机器人模式遥控器通道
typedef enum
{
    ROBOT_ZERO_FORCE = 0,       //零力模式
    ROBOT_INIT,                 //初始化模式
    ROBOT_CALIBRATION,          //校准模式
    // ROBOT_ABSOLUTE_CONTROL,     //绝对控制模式
    // ROBOT_RELATIVE_CONTROL,     //相对控制模式
    ROBOT_FOLLOWED_CONTROL,       //跟随控制模式
    ROBOT_INDEPENDENT_CONTROL,  //独立控制模式
    ROBOT_TEST,                 //测试模式 
}Robot_main_mode_e;//机器人主模式


typedef struct 
{
    /* data */
    Robot_main_mode_e main_mode; //机器人主模式
    Robot_main_mode_e last_main_mode; //上一次机器人主模式
    const RC_ctrl_t *robot_rc_ctrl;//遥控器指针
}Robot_t;//机器人结构体
extern const Robot_t *get_robot_point(void);
/* USER CODE END Includes Form C Files*/
#ifdef __cplusplus
}
#endif

#endif
