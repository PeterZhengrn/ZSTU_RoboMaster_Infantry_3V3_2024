/**
 * @file default_task.c
 * @brief 	according to the value of the remote control, decide the main task of the robot
 * 			根据遥控器的值，决定机器人主任务
 * @history
 * Version     Date            Author          Modification
 * V1.0.0      Oct-26-2023     Higashiki       初步完成

*/



#include "default_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
// #include "bsp_servo_pwm.h"

/* USER CODE BEGIN Header */
/**
* @brief Function implementing the Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
Robot_t robot;
static void Robot_init(Robot_t *robot_init);
static void robot_mode_set(Robot_t *robot_set);
void default_task(void const * argument)
{
  /* USER CODE BEGIN  */
	Robot_init(&robot);//初始化机器人结构体
  /* Infinite loop */
	while(1)
	{
		robot_mode_set(&robot);
		vTaskDelay(1);
	}
  /* USER CODE END */
}


static void Robot_init(Robot_t *robot_init)
{
	if(robot_init == NULL)
	{
		return;
	}

	robot_init->main_mode = ROBOT_ZERO_FORCE;//机器人主模式是无力模式
	robot_init->last_main_mode = ROBOT_ZERO_FORCE;//上一次机器人主模式是无力模式
	//初始化机器人结构体
	robot_init->robot_rc_ctrl = get_remote_control_point();//获取遥控器数据指针
	
}
static void robot_mode_set(Robot_t *robot_set)
{
	if(robot_set == NULL)
	{
		return;
	}
	robot_set->last_main_mode = robot_set->main_mode;//记录上一次机器人主模式
	if(switch_is_down(robot_set->robot_rc_ctrl->rc.s[ROBOT_MODE_CHANNEL]))
	{
		//The mode of the robot is zero force mode
		robot_set->main_mode=ROBOT_ZERO_FORCE;
	}
	else if(switch_is_up(robot_set->robot_rc_ctrl->rc.s[ROBOT_MODE_CHANNEL]))
	{
		//The mode of the robot is absolute control mode
		robot_set->main_mode=ROBOT_FOLLOWED_CONTROL;
	}
	else if(switch_is_mid(robot_set->robot_rc_ctrl->rc.s[ROBOT_MODE_CHANNEL]))
	{
		//The mode of the robot is relative control mode
		robot_set->main_mode=ROBOT_INDEPENDENT_CONTROL;
	}
}
/**
 * @brief  机器人主指针
 * @param  none
 * @retval 机器人主指针
 */
const Robot_t *get_robot_point(void)
{
	return &robot;
}



