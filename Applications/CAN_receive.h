#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan2 //���̵��CAN
#define STEER_CAN hcan2   //���ֵ��CAN
#define GIMBAL_CAN hcan1  //��̨���CAN
#define SHOOT_CAN hcan1   //�������CAN

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
} can_msg_id_e;

typedef enum
{
    CAN1_MOTOR_ALL_ID = 0x200,  // can1�󽮼��ٵ������ID
    CAN1_FRIC_M1_ID = 0x201,       //Ħ����1���ID
    CAN1_FRIC_M2_ID = 0x202,       //Ħ����2���ID
    CAN1_TRIGGER_MOTOR_ID = 0x203, //�������ID

    CAN1_GIMBAL_ALL_ID = 0x1FF, // can1����̨�������ID
    CAN1_YAW_MOTOR_ID = 0x205, //��̨yaw���ID
    CAN1_PITCH_G1_MOTOR_ID = 0x206, //��̨pitch_g1���ID
    CAN1_PITCH_G2_MOTOR_ID = 0x207, //��̨pitch_g2���ID
} can1_msg_id_e;

typedef enum
{
    CAN2_CHASSIS_ALL_ID = 0x200, //���̵������ID
    CAN2_3508_M1_ID = 0x201,     //���̵��M1
    CAN2_3508_M2_ID = 0x202,     //���̵��M2
    CAN2_3508_M3_ID = 0x203,     //���̵��M3
    CAN2_3508_M4_ID = 0x204,     //���̵��M4


    CAN1_STEER_ALL_ID = 0x1FF,   //ת��������ID
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
extern void CAN_cmd_steer(int16_t steer1,int16_t steer2,int16_t steer3,int16_t steer4);
// extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
// extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_steer_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_fric1_motor_measure_point(void);
extern const motor_measure_t *get_fric2_motor_measure_point(void);

#endif
