#include "shoot_task.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "math.h"
#include "pid.h"
shoot_control_t shoot_control; //�������

static void shoot_feedback_update(void);
static void trigger_motor_turn_back(void);
static void shoot_bullet_control(void);

/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_init(void)
{
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.shoot_last_mode = SHOOT_STOP;
	//ң����ָ��
	shoot_control.shoot_rc = get_remote_control_point();
	//���ָ��
	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
	//Ħ�������ݸ���
	shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
	shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
	//������ݸ���
	shoot_feedback_update();
	//������PID��ʼ��
	PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
	//Ħ���ֵ��1PID��ʼ��
	PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	//Ħ���ֵ��2PID��ʼ��
	PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	shoot_control.angle = shoot_control.shoot_motor_measure->total_angle * MOTOR_ECD_TO_ANGLE;
	shoot_control.given_current = 0;
	shoot_control.move_flag = 0;
	shoot_control.set_angle = shoot_control.angle;
	shoot_control.speed = 0.0f;
	shoot_control.speed_set = 0.0f;
}

/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;
	//�����ֵ���ٶ��˲�һ��
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
	shoot_control.speed = speed_fliter_3;

	shoot_control.fric1_speed = shoot_control.fric1_motor_measure->speed_rpm;
	shoot_control.fric2_speed = shoot_control.fric2_motor_measure->speed_rpm;

	//���������Ƕ�
	shoot_control.angle = shoot_control.shoot_motor_measure->total_angle * MOTOR_ECD_TO_ANGLE;
	//��갴��
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
	//������ʱ
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

	//��������µ�ʱ���ʱ
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
 * @brief          ���״̬������
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_UP;
	//�ϲ��жϣ� һ�ο������ٴιر�
	if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
	}
	else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}

	//����ж���Ҫ���ڼ��̿���Ħ���ֵĿ���
	//�����е��� ����ʹ�ü��̿���Ħ����
	if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
	}
	//�����е��� ����ʹ�ü��̹ر�Ħ����
	else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}

	//���Ħ�����Ѿ�ת�����ˣ��ͽ����ӵ�׼��״̬
	// if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
	if (shoot_control.shoot_mode == SHOOT_READY_FRIC) //���ﲻ�걸����Ҫ��һ��Ħ���ּ�������жϣ�Ħ���ֵ���ָ��ת�٣�
	{
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
	//�����ӵ�����׼���׶Σ������ӵ������������������״̬
	// else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		shoot_control.shoot_mode = SHOOT_READY;
	}
	// //�����������״̬�������ӵ�û�о����������ӵ�����׼��״̬
	// else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
	// {
	//     shoot_control.shoot_mode = SHOOT_READY_BULLET;
	// }
	//�����������״̬
	else if (shoot_control.shoot_mode == SHOOT_READY)
	{
		//�²�һ�λ�����갴��һ�Σ��������״̬
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
		//��곤��һֱ�������״̬ ��������
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
	//ǹ���������ߣ�����������״̬
	// get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
	// if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
	// {
	//     if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	//     {
	//         shoot_control.shoot_mode =SHOOT_READY_BULLET;
	//     }
	// }
	//�����̨״̬�� ����״̬���͹ر����
	// if (gimbal_cmd_to_shoot_stop())
	// {
	//     shoot_control.shoot_mode = SHOOT_STOP;
	// }
	last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          �������ѭ��
 * @param[in]      void
 * @retval         void
 */
static void shoot_control_loop(void)
{
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		//���ò����ֵ��ٶ�
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		//���ò����ֵ��ٶ�
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		if (shoot_control.key == SWITCH_TRIGGER_OFF)
		{
			//���ò����ֵĲ����ٶ�,��������ת��ת����
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
		//���ò����ֵ��ٶ�
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
		//���ò����ֵĲ����ٶ�,��������ת��ת����
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
		// //����Ħ���ֵ��1PID
		// PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_target_speed);
		// shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
		// //����Ħ���ֵ��2PID
		// PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_target_speed);
		// shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
	}
	else
	{
		//        shoot_laser_on(); //���⿪��
		shoot_control.fric1_target_speed = -FRIC_SPEED;
		shoot_control.fric2_target_speed = FRIC_SPEED;
		//���㲦���ֵ��PID
		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set * TRIGGER_REVERSE);
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

		//����Ħ���ֵ��1PID
		PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_target_speed);
		shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
		// shoot_control.fric1_given_current = -FRIC_SPEED;
		//����Ħ���ֵ��2PID
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
 * @brief ��ת�ж��Լ��������
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
 * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
	//˳ʱ��Ѱ�������PI/4�ĽǶȲ�����
	if (shoot_control.move_flag == 0)
	{
		//У׼�жϺ󷵻���ΪPI_FOUR��������˳ʱ������ĵ�
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
	//����Ƕ��ж�
	if (fabs(rad_format(shoot_control.set_angle - shoot_control.angle)) > 0.05f)
	{
		//û����һֱ������ת�ٶ�
		shoot_control.trigger_speed_set = TRIGGER_SPEED;
		trigger_motor_turn_back();
	}
	else
	{
		//У׼�ж�
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
	shoot_init(); //��ʼ��
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
