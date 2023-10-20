/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1
#define SHOOT_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
} can_msg_id_e;

typedef enum
{
    CAN1_MOTOR_ALL_ID = 0x200,  // can1�󽮼��ٵ������ID
    CAN1_GIMBAL_ALL_ID = 0x1FF, // can1����̨�������ID

    CAN1_FRIC_M1_ID = 0x201,       //Ħ����1���ID
    CAN1_FRIC_M2_ID = 0x202,       //Ħ����2���ID
    CAN1_TRIGGER_MOTOR_ID = 0x203, //�������ID

    CAN1_YAW_MOTOR_ID = 0x205, //��̨yaw���ID
    CAN1_PIT_MOTOR_ID = 0x206, //��̨pitch���ID

} can1_msg_id_e;

typedef enum
{
    CAN2_CHASSIS_ALL_ID = 0x200, //�󽮼��ٵ������ID
    CAN1_STEER_ALL_ID = 0x1FF,   //����̨���(��)����ID

    CAN2_3508_M1_ID = 0x201,
    CAN2_3508_M2_ID = 0x202,
    CAN2_3508_M3_ID = 0x203,
    CAN2_3508_M4_ID = 0x204,
    CAN2_STEER_M1_MOTOR_ID = 0x205, //ת����M1
    CAN2_STEER_M2_MOTOR_ID = 0x206, //ת����M2
    CAN2_STEER_M3_MOTOR_ID = 0x207, //ת����M3
    CAN2_STEER_M4_MOTOR_ID = 0x208, //ת����M4
} can2_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
    uint16_t offset_angle; //�������
    int32_t round_cnt;
    int32_t total_angle;
    uint8_t msg_cnt;
} motor_measure_t;

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_shoot(int16_t fric1, int16_t fric2, int16_t trigger, int16_t rev);
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_fric1_motor_measure_point(void);
extern const motor_measure_t *get_fric2_motor_measure_point(void);

#endif
