#include "shoot_task.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "math.h"
#include "pid.h"
shoot_control_t shoot_control; //射击数据

static void shoot_feedback_update(void);
static void trigger_motor_turn_back(void);
static void shoot_bullet_control(void);

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(void)
{
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.shoot_last_mode = SHOOT_STOP;
	//遥控器指针
	shoot_control.shoot_rc = get_remote_control_point();
	//电机指针
	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
	//摩擦轮数据更新
	shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
	shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
	//射击数据更新
	shoot_feedback_update();
	//拨弹轮PID初始化
	PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
	//摩擦轮电机1PID初始化
	PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	//摩擦轮电机2PID初始化
	PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	shoot_control.angle = shoot_control.shoot_motor_measure->total_angle * MOTOR_ECD_TO_ANGLE;
	shoot_control.given_current = 0;
	shoot_control.move_flag = 0;
	shoot_control.set_angle = shoot_control.angle;
	shoot_control.speed = 0.0f;
	shoot_control.speed_set = 0.0f;
}

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;
	//拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
	shoot_control.speed = speed_fliter_3;

	shoot_control.fric1_speed = shoot_control.fric1_motor_measure->speed_rpm;
	shoot_control.fric2_speed = shoot_control.fric2_motor_measure->speed_rpm;

	//计算输出轴角度
	shoot_control.angle = shoot_control.shoot_motor_measure->total_angle * MOTOR_ECD_TO_ANGLE;
	//鼠标按键
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
	//长按计时
	if (shoot_control.press_l)
	{
		if (shoot_control.press_l_time < PRESS_LONG_TIME)
		{
			shoot_control.press_l_time++;
		}
	}
	else
	{
		shoot_control.press_l_time = 0;
	}

	if (shoot_control.press_r)
	{
		if (shoot_control.press_r_time < PRESS_LONG_TIME)
		{
			shoot_control.press_r_time++;
		}
	}
	else
	{
		shoot_control.press_r_time = 0;
	}

	//射击开关下档时间计时
	if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
	{

		if (shoot_control.rc_s_time < RC_S_LONG_TIME)
		{
			shoot_control.rc_s_time++;
		}
	}
	else
	{
		shoot_control.rc_s_time = 0;
	}
}
/**
 * @brief          射击状态机设置
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_UP;
	//上拨判断， 一次开启，再次关闭
	if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
	}
	else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}

	//这个判断主要用于键盘控制摩擦轮的开关
	//处于中档， 可以使用键盘开启摩擦轮
	if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
	}
	//处于中档， 可以使用键盘关闭摩擦轮
	else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}

	//如果摩擦轮已经转起来了，就进入子弹准备状态
	// if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
	if (shoot_control.shoot_mode == SHOOT_READY_FRIC) //这里不完备，需要加一个摩擦轮加速完成判断（摩擦轮到达指定转速）
	{
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
	//处于子弹发射准备阶段，并且子弹就绪，进入射击就绪状态
	// else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		shoot_control.shoot_mode = SHOOT_READY;
	}
	// //处于射击就绪状态，但是子弹没有就绪，进入子弹发射准备状态
	// else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
	// {
	//     shoot_control.shoot_mode = SHOOT_READY_BULLET;
	// }
	//处于射击就绪状态
	else if (shoot_control.shoot_mode == SHOOT_READY)
	{
		//下拨一次或者鼠标按下一次，进入射击状态
		if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
		{
			shoot_control.shoot_mode = SHOOT_BULLET;
		}
	}
	else if (shoot_control.shoot_mode == SHOOT_DONE)
	{
		// if(shoot_control.key == SWITCH_TRIGGER_OFF)
		// {
		//     shoot_control.key_time++;
		//     if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
		//     {
		//         shoot_control.key_time = 0;
		//         shoot_control.shoot_mode = SHOOT_READY_BULLET;
		//     }
		// }
		// else
		// {
		//     shoot_control.key_time = 0;
		//     shoot_control.shoot_mode = SHOOT_BULLET;
		// }
		shoot_control.shoot_mode = SHOOT_READY;
	}

	if (shoot_control.shoot_mode > SHOOT_READY_FRIC)
	{
		//鼠标长按一直进入射击状态 保持连发
		if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
		{
			shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			shoot_control.shoot_last_mode = SHOOT_CONTINUE_BULLET;
		}
		else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_READY_BULLET;
		}
	}
	//枪口热量过高，进入射击完成状态
	// get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
	// if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
	// {
	//     if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	//     {
	//         shoot_control.shoot_mode =SHOOT_READY_BULLET;
	//     }
	// }
	//如果云台状态是 无力状态，就关闭射击
	// if (gimbal_cmd_to_shoot_stop())
	// {
	//     shoot_control.shoot_mode = SHOOT_STOP;
	// }
	last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          射击控制循环
 * @param[in]      void
 * @retval         void
 */
static void shoot_control_loop(void)
{
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		//设置拨弹轮的速度
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		//设置拨弹轮的速度
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		if (shoot_control.key == SWITCH_TRIGGER_OFF)
		{
			//设置拨弹轮的拨动速度,并开启堵转反转处理
			shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
			trigger_motor_turn_back();
		}
		else
		{
			shoot_control.trigger_speed_set = 0.0f;
			shoot_control.speed_set = 0.0f;
		}
		shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
		shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY)
	{
		//设置拨弹轮的速度
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
		shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
		shoot_bullet_control();
	}
	else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	{
		//设置拨弹轮的拨动速度,并开启堵转反转处理
		shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
		trigger_motor_turn_back();
	}
	else if (shoot_control.shoot_mode == SHOOT_DONE)
	{
		shoot_control.speed_set = 0.0f;
	}

	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		//        shoot_laser_off();
		shoot_control.given_current = 0;
		shoot_control.fric1_target_speed = 0.0f;
		shoot_control.fric2_target_speed = 0.0f;
		shoot_control.fric1_given_current = 95;
		shoot_control.fric2_given_current = -95;
		// //计算摩擦轮电机1PID
		// PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_target_speed);
		// shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
		// //计算摩擦轮电机2PID
		// PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_target_speed);
		// shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
	}
	else
	{
		//        shoot_laser_on(); //激光开启
		shoot_control.fric1_target_speed = -FRIC_SPEED;
		shoot_control.fric2_target_speed = FRIC_SPEED;
		//计算拨弹轮电机PID
		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set * TRIGGER_REVERSE);
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

		//计算摩擦轮电机1PID
		PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_target_speed);
		shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
		// shoot_control.fric1_given_current = -FRIC_SPEED;
		//计算摩擦轮电机2PID
		PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_target_speed);
		shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
		// shoot_control.fric2_given_current = FRIC_SPEED;
		if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
		{
			shoot_control.given_current = 0;
		}
	}
}
/**
 * @brief 堵转判断以及处理程序、
 * @param[in] void
 * @retval void
 */
static void trigger_motor_turn_back(void)
{
	if (shoot_control.block_time < BLOCK_TIME)
	{
		shoot_control.speed_set = shoot_control.trigger_speed_set;
	}
	else
	{
		shoot_control.speed_set = -shoot_control.trigger_speed_set;
	}

	if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
	{
		shoot_control.block_time++;
		shoot_control.reverse_time = 0;
	}
	else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
	{
		shoot_control.reverse_time++;
	}
	else
	{
		shoot_control.block_time = 0;
	}
}
/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
	//顺时针寻找最近的PI/4的角度并拨动
	if (shoot_control.move_flag == 0)
	{
		//校准判断后返回以为PI_FOUR整数倍的顺时针最近的点
		if (shoot_control.shoot_last_mode == SHOOT_BULLET)
		{
			shoot_control.set_angle = rad_format(rad_format(round(shoot_control.angle / PI_FOUR) * PI_FOUR) + PI_FOUR * TRIGGER_REVERSE);
		}
		else
		{
			shoot_control.set_angle = rad_format(rad_format(round(shoot_control.angle / PI_FOUR) * PI_FOUR) + PI_EIGHT * TRIGGER_REVERSE);
		}
		shoot_control.move_flag = 1;
	}
	//到达角度判断
	if (fabs(rad_format(shoot_control.set_angle - shoot_control.angle)) > 0.05f)
	{
		//没到达一直设置旋转速度
		shoot_control.trigger_speed_set = TRIGGER_SPEED;
		trigger_motor_turn_back();
	}
	else
	{
		//校准判断
		if (shoot_control.shoot_last_mode == SHOOT_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_DONE;
		}
		else
		{
			shoot_control.shoot_last_mode = SHOOT_BULLET;
		}
		shoot_control.move_flag = 0;
	}
}

/* USER CODE BEGIN Header */
/**
 * @brief Function implementing the Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_shoot_task */
void shoot_task(void const *argument)
{
	/* USER CODE BEGIN  */
	/* Infinite loop */
	shoot_init(); //初始化
	while (1)
	{
		shoot_set_mode();
		shoot_feedback_update();
		shoot_control_loop();
		// CAN_cmd_shoot(shoot_control.fric1_given_current, shoot_control.fric2_given_current, shoot_control.given_current,0);
		CAN_cmd_shoot(shoot_control.fric1_given_current, shoot_control.fric2_given_current, shoot_control.given_current, 0);
		// CAN_cmd_shoot(0, 0, shoot_control.given_current,0);
		// CAN_cmd_shoot(-FRIC_SPEED, FRIC_SPEED, shoot_control.given_current,0);
		// CAN_cmd_shoot(0,0,0,0);
		vTaskDelay(SHOOT_CONTROL_TIME);
	}
	/* USER CODE END */
}
