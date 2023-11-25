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
//     //��������
//     CHASSIS_MECANUM = 0, //�����ķ��
//     CHASSIS_OMNI,       //ȫ����
//     CHASSIS_STEER,//����
// }chassis_type_e;//��������

// typedef enum
// {
//     //��̨����
//     GIMBAL_SINGLE=0,     //����̨
//     GIMBAL_DOUBLE, //˫��̨
// }gimbal_type_e;//��̨����
#define ROBOT_MODE_CHANNEL 0 //������ģʽң����ͨ��
typedef enum
{
    ROBOT_ZERO_FORCE = 0,       //����ģʽ
    ROBOT_INIT,                 //��ʼ��ģʽ
    ROBOT_CALIBRATION,          //У׼ģʽ
    // ROBOT_ABSOLUTE_CONTROL,     //���Կ���ģʽ
    // ROBOT_RELATIVE_CONTROL,     //��Կ���ģʽ
    ROBOT_FOLLOWED_CONTROL,       //�������ģʽ
    ROBOT_INDEPENDENT_CONTROL,  //��������ģʽ
    ROBOT_TEST,                 //����ģʽ 
}Robot_main_mode_e;//��������ģʽ


typedef struct 
{
    /* data */
    Robot_main_mode_e main_mode; //��������ģʽ
    Robot_main_mode_e last_main_mode; //��һ�λ�������ģʽ
    const RC_ctrl_t *robot_rc_ctrl;//ң����ָ��
}Robot_t;//�����˽ṹ��
extern const Robot_t *get_robot_point(void);
/* USER CODE END Includes Form C Files*/
#ifdef __cplusplus
}
#endif

#endif
