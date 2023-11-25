#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "math.h"
/*-----------------------------------------------------------------------------------------------------------*/
static void chassis_init(chassis_move_t *chassis_move_init);//https://gitee.com/Thepeter/new_rm.git
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_control_vector_set(chassis_move_t *chassis_control_vector_set);
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_follow_gimbal_yaw);
static void chassis_follow_chassis_yaw_control(chassis_move_t *chassis_follow_chassis_yaw);
static void chassis_independent_control(chassis_move_t *chassis_independent);
static void chassis_raw_control(chassis_move_t *chassis_raw);
static void chassis_zero_force_control(chassis_move_t *chassis_zero_force);
void AGV_angle_calc( fp32 vx_set,  fp32 vy_set,  fp32 wz_set, fp32 *chassis_6020_setangle) ;
static void chassis_vector_to_Steer_wheel_speed( fp32 vx_set,  fp32 vy_set,  fp32 wz_set, fp32 *wheel_speed);
//static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
/*------------------------------------------------------------------------------------------------------------*/
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
chassis_move_t chassis_move;
/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const * argument)
{
    /* USER CODE BEGIN chassis_task */
    chassis_init(&chassis_move);
    /* Infinite loop */
    while(1)
    {
        chassis_set_mode(&chassis_move);/////
        chassis_mode_change_control_transit(&chassis_move);/////
        chassis_feedback_update(&chassis_move);
        chassis_control_vector_set(&chassis_move);//////
        chassis_control_loop(&chassis_move);
        // osDelay(2);
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
        #if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
    /* USER CODE END chassis_task */
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if(chassis_move_init == NULL)
    {
        return;
    }
    //chassis type
    //��������
    chassis_move_init->chassis_type = CHASSIS_TYPE;//��������

    //����x�����ٶȵ�ͨ�˲�����
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    //����y�����ٶȵ�ͨ�˲�����
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //robot structure point
    //�����˽ṹ��ָ��
    chassis_move_init->robot = get_robot_point();
    //in beginning�� chassis mode is raw 
    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;//���̵������ֱ�ӷ��͵�CAN����
    chassis_move_init->last_chassis_mode = CHASSIS_VECTOR_RAW;//���̿���״̬Ϊԭʼ
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();

    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    if(chassis_move_init->chassis_type == CHASSIS_STEER)
    {
        //get chassis and steer motor measure point
        //��ȡ���̺�ת������������ָ��
        for (uint8_t i = 0; i < 4; i++)
        {
            //���̵����������ָ��
            chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
            //ת������������ָ��
            chassis_move_init->motor_steer[i].steer_motor_measure = get_steer_motor_measure_point(i);
               //initialize motor speed PID
                //��ʼ��PID 
                for (uint8_t i = 0; i < 4; i++)
                {
                    PID_init(&chassis_move_init->motor_speed_pid[i], 
                              PID_POSITION, 
                              M3505_MOTOR_SPEED_PID_KP, 
                              M3505_MOTOR_SPEED_PID_KI, 
                              M3505_MOTOR_SPEED_PID_KD,
                              M3505_MOTOR_SPEED_PID_MAX_OUT, 
                              M3505_MOTOR_SPEED_PID_MAX_IOUT);
                }
                   for (uint8_t i = 0; i < 4; i++)
                {
                    PID_init(&chassis_move_init->motor_steer[i], 
                              PID_POSITION, 
                              M3505_MOTOR_SPEED_PID_KP, 
                              M3505_MOTOR_SPEED_PID_KI, 
                              M3505_MOTOR_SPEED_PID_KD,
                              M3505_MOTOR_SPEED_PID_MAX_OUT, 
                              M3505_MOTOR_SPEED_PID_MAX_IOUT);
                }
        }


    }

 


    // initialize angle PID
    // ��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, 
              PID_POSITION, 
              CHASSIS_FOLLOW_GIMBAL_PID_KP, 
              CHASSIS_FOLLOW_GIMBAL_PID_KI, 
              CHASSIS_FOLLOW_GIMBAL_PID_KD,
              CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, 
              CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //first order low-pass filter  replace ramp function
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, 
                            CHASSIS_CONTROL_TIME, 
                            chassis_x_order_filter);

    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, 
                            CHASSIS_CONTROL_TIME, 
                            chassis_y_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);    

}
/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
  if(chassis_move_mode == NULL)
  {
    return;
  }



  //chassis mode state machine:
  //���̿���״̬��:

  if(chassis_move_mode->robot->main_mode == ROBOT_ZERO_FORCE)
  {
    //The mode of the robot is zero force mode
    //��������ģʽ������ģʽ
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//���̵������ֱ�ӷ��͵�CAN����
  }

  else if(chassis_move_mode->robot->main_mode == ROBOT_FOLLOWED_CONTROL)
  {
    // chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;//���̻������̨��ԽǶ�
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;//�����е��̽Ƕȿ��Ʊջ�
  }
  else if(chassis_move_mode->robot->main_mode == ROBOT_INDEPENDENT_CONTROL)
  {
    // chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;//�����е��̽Ƕȿ��Ʊջ�
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_INDENPENDENT;//��������ת�ٶȿ���
  }
  else if(chassis_move_mode->robot->main_mode == ROBOT_TEST)
  {
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//���̵������ֱ�ӷ��͵�CAN����
  }
  else if(chassis_move_mode->robot->main_mode == ROBOT_CALIBRATION)
  {
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//���̵������ֱ�ӷ��͵�CAN����
  }

}

/**
  * @brief          when chassis mode change, some param should be changed
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
  if (chassis_move_transit == NULL)
  {
      return;
  }

  if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
  {
      return;
  }


  //change to follow gimbal angle mode
  //���������̨ģʽ
  if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
  {
      chassis_move_transit->chassis_relative_angle_set = 0.0f;
  }
  //change to follow chassis yaw angle
  //���������̽Ƕ�ģʽ
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
  {
      chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }
  //change to no follow angle
  //���벻������̨ģʽ
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_INDENPENDENT) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_INDENPENDENT)
  {
      chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }
  //��������ģʽ
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_ZERO_FORCE) && chassis_move_transit->chassis_mode == CHASSIS_ZERO_FORCE)
  {
      for (uint8_t i = 0; i < 4; i++)
      {
          PID_clear(&chassis_move_transit->motor_speed_pid[i]);
          chassis_move_transit->motor_chassis[i].give_current = 0.0f;
      }
  }
  //record last chassis control mode
  //��¼��һ�ε��̿���ģʽ
  chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    // chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    // chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));    
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
static void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
 * @brief           according to chassis control mode, choose the way to calculate chassis speed
 * 
 * @param[in]       chassis_move_vector_set: "chassis_move" valiable point
 * @retval          none
*/
/**
 * @brief           ���ݵ��̿���ģʽ��ѡ���������ٶȵķ�ʽ
 * 
 * @param[in]       chassis_move_vector_set: "chassis_move"����ָ��
 * @retval          none
*/
static void chassis_control_vector_set(chassis_move_t *chassis_control_vector_set)
{
    if(chassis_control_vector_set == NULL)
    {
        return;
    }

    if(chassis_control_vector_set->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_follow_gimbal_yaw_control(chassis_control_vector_set);
    }
    else if(chassis_control_vector_set->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_follow_chassis_yaw_control(chassis_control_vector_set);
    }
    else if(chassis_control_vector_set->chassis_mode == CHASSIS_VECTOR_INDENPENDENT)
    {
        chassis_independent_control(chassis_control_vector_set);
    }
    else if(chassis_control_vector_set->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_raw_control(chassis_control_vector_set);
    }
    else if(chassis_control_vector_set->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(chassis_control_vector_set);
    }

}



/**
 * @brief           chassis follow gimbal yaw angle, calculate chassis vertical and horizontal speed set-point
 * 
 * @param[in]       chassis_follow_gimbal_yaw: "chassis_move" valiable point
 * @retval          none
*/
/**
 * @brief           ���̸�����̨yaw�Ƕȿ��ƣ����������ƽ���ٶȺ���ת���ٶ�
 * 
 * @param[in]       chassis_follow_gimbal_yaw: "chassis_move"����ָ��
 * @retval          none
*/
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_follow_gimbal_yaw)
{
    if(chassis_follow_gimbal_yaw == NULL)
    {
        return;
    }
    fp32 vx_set=0, vy_set=0, angle_set=0;
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_follow_gimbal_yaw);
    angle_set = 0.0f;
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
    //rotate chassis direction, make sure vertial direction follow gimbal 
    //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
    // sin_yaw = arm_sin_f32(-chassis_follow_gimbal_yaw->chassis_yaw_motor->relative_angle);
    // cos_yaw = arm_cos_f32(-chassis_follow_gimbal_yaw->chassis_yaw_motor->relative_angle);
    // sin_yaw = sin(-chassis_follow_gimbal_yaw->chassis_yaw_motor->relative_angle);
    // cos_yaw = cos(-chassis_follow_gimbal_yaw->chassis_yaw_motor->relative_angle);    
    chassis_follow_gimbal_yaw->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
    chassis_follow_gimbal_yaw->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
    //set control relative angle  set-point
    //���ÿ��������̨�Ƕ�
    chassis_follow_gimbal_yaw->chassis_relative_angle_set = rad_format(angle_set);
    //calculate ratation speed
    //������תPID���ٶ�
    // chassis_follow_gimbal_yaw->wz_set = -PID_calc(&chassis_follow_gimbal_yaw->chassis_angle_pid, chassis_follow_gimbal_yaw->chassis_yaw_motor->relative_angle, chassis_follow_gimbal_yaw->chassis_relative_angle_set);
    //speed limit
    //�ٶ��޷�
    chassis_follow_gimbal_yaw->vx_set = fp32_constrain(chassis_follow_gimbal_yaw->vx_set, chassis_follow_gimbal_yaw->vx_min_speed, chassis_follow_gimbal_yaw->vx_max_speed);
    chassis_follow_gimbal_yaw->vy_set = fp32_constrain(chassis_follow_gimbal_yaw->vy_set, chassis_follow_gimbal_yaw->vy_min_speed, chassis_follow_gimbal_yaw->vy_max_speed);
}


/**
 * @brief           chassis follow chassis yaw angle, calculate chassis vertical and horizontal speed set-point
 * @param[in]       chassis_follow_chassis_yaw: "chassis_move" valiable point
 * @retval          none
*/
/**
 * @brief           ���̸������yaw�Ƕȿ��ƣ����������ƽ���ٶȺ���ת���ٶ�
 * @param[in]       chassis_follow_chassis_yaw: "chassis_move"����ָ��
 * @retval          none
*/
static void chassis_follow_chassis_yaw_control(chassis_move_t *chassis_follow_chassis_yaw)
{
    if(chassis_follow_chassis_yaw == NULL)
    {
        return;
    }
    fp32 vx_set=0.0f, vy_set=0.0f, angle_set=0.0f,delat_angle = 0.0f;
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_follow_chassis_yaw);
    angle_set = rad_format(chassis_follow_chassis_yaw->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_follow_chassis_yaw->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
    //set chassis yaw angle set-point
    //���õ��̿��ƵĽǶ�
    chassis_follow_chassis_yaw->chassis_yaw_set = rad_format(angle_set);
    delat_angle = rad_format(chassis_follow_chassis_yaw->chassis_yaw_set - chassis_follow_chassis_yaw->chassis_yaw);
    //calculate rotation speed
    //������ת�Ľ��ٶ�
    chassis_follow_chassis_yaw->wz_set = PID_calc(&chassis_follow_chassis_yaw->chassis_angle_pid, 0.0f, delat_angle);
    //speed limit
    //�ٶ��޷�
    chassis_follow_chassis_yaw->vx_set = fp32_constrain(vx_set, chassis_follow_chassis_yaw->vx_min_speed, chassis_follow_chassis_yaw->vx_max_speed);
    chassis_follow_chassis_yaw->vy_set = fp32_constrain(vy_set, chassis_follow_chassis_yaw->vy_min_speed, chassis_follow_chassis_yaw->vy_max_speed);
}


/**
 * @brief           chassis independent control, calculate chassis vertical and horizontal speed set-point
 * @param[in]       chassis_independent: "chassis_move" valiable point
 * @retval          none
*/
/**
 * @brief           ���̶������ƣ����������ƽ���ٶȺ���ת���ٶ�
 * @param[in]       chassis_independent: "chassis_move"����ָ��
 * @retval          none
*/
static void chassis_independent_control(chassis_move_t *chassis_independent)
{
    if(chassis_independent == NULL)
    {
        return;
    }
    fp32 vx_set=0.0f, vy_set=0.0f;
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_independent);
    chassis_independent->wz_set = -CHASSIS_WZ_RC_SEN * chassis_independent->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
    chassis_independent->vx_set = fp32_constrain(vx_set, chassis_independent->vx_min_speed, chassis_independent->vx_max_speed);
    chassis_independent->vy_set = fp32_constrain(vy_set, chassis_independent->vy_min_speed, chassis_independent->vy_max_speed);
}


/**
 * @brief           chassis raw control, calculate chassis vertical and horizontal speed set-point
 * @param[in]       chassis_raw: "chassis_move" valiable point
 * @retval          none
*/
/**
 * @brief           ����ԭʼ���ƣ����������ƽ���ٶȺ���ת���ٶ�
 * @param[in]       chassis_raw: "chassis_move"����ָ��
 * @retval          none
*/
static void chassis_raw_control(chassis_move_t *chassis_raw)
{
    if(chassis_raw == NULL)
    {
        return;
    }
    fp32 vx_set, vy_set;
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_raw);
    chassis_raw->vx_set = vx_set;
    chassis_raw->vy_set = vy_set;
    chassis_raw->wz_set = chassis_raw->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_WZ_RC_SEN;
}


static void chassis_zero_force_control(chassis_move_t *chassis_zero_force)
{
    if(chassis_zero_force == NULL)
    {
        return;
    }
    chassis_zero_force->vx_set = 0.0f;
    chassis_zero_force->vy_set = 0.0f;
    chassis_zero_force->wz_set = 0.0f;
}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
// static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
// {
//     //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
//     //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
//     wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//     wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//     wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//     wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
// }



/**
  * @brief  ������̺�������Ŀ��Ƕ�
  * @param  speed ����������ٶ� 
  * @param  out_angle 6020Ŀ��Ƕ�
  * @retval 
  * @attention
  */

void AGV_angle_calc( fp32 vx_set,  fp32 vy_set,  fp32 wz_set, fp32 *chassis_6020_setangle) 
{

    fp32 Radius=1;
    fp32 wheel_angle[4]; //6020������Ŀ��Ƕ�

    fp64 atan_angle[4];   
	  //6020Ŀ��Ƕȼ���
    //Radius���ӵ����ĵľ���
    //0.707107f �Խ��߷����ϵ��
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//��ֹ����Ϊ��
    {
      atan_angle[0]=atan2((vx_set - wz_set*Radius*0.707107f),(vy_set + wz_set*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2((vx_set - wz_set*Radius*0.707107f),(vy_set- wz_set*Radius*0.707107f))*180.0f/PI;
      atan_angle[2]=atan2((vx_set + wz_set*Radius*0.707107f),(vy_set - wz_set*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2((vx_set + wz_set*Radius*0.707107f),(vy_set + wz_set*Radius*0.707107f))*180.0f/PI;	
    }  

		wheel_angle[0] =   (fp32)(atan_angle[0]*22.75);
		wheel_angle[1] =   (fp32)(atan_angle[1]*22.75);
		wheel_angle[2] =   (fp32)(atan_angle[2]*22.75);
		wheel_angle[3] =  (fp32)(atan_angle[3]*22.75);
    wheel_angle[0]=loop_fp32_constrain(wheel_angle[0],0,8192);//�����ͽǶȻػ�
    wheel_angle[1]=loop_fp32_constrain(wheel_angle[1],0,8192);//�����ͽǶȻػ�
    wheel_angle[2]=loop_fp32_constrain(wheel_angle[2],0,8192);//�����ͽǶȻػ�
    wheel_angle[3]=loop_fp32_constrain(wheel_angle[3],0,8192);//�����ͽǶȻػ�
  //�������Ҫ�ӿ���ֵı仯�ٶȿ��Լ�һ�������������ҽǶȵ����ת���Ƿ�ת�ĸ�����ĺ���������һ����ת����
  //Ȼ���ڵ����ǳ�
  chassis_6020_setangle[0]= wheel_angle[1]  ; 
  chassis_6020_setangle[1]= wheel_angle[1]  ;
  chassis_6020_setangle[2] =wheel_angle[2]  ;
  chassis_6020_setangle[3]=wheel_angle[3]  ;
    
	
}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_Steer_wheel_speed( fp32 vx_set,  fp32 vy_set,  fp32 wz_set, fp32 *wheel_speed)
{
      fp32 wheel_rpm_ratio;
    fp32 Radius=1;////2023/11/24���������Ҫ�䣬��������ʱ��Ҫ�ģ�����û�ж���,��������Ƕ��ֵ����ĵľ���
    wheel_rpm_ratio = 1;//2023/11/24���������Ҫ�䣬��������ʱ��Ҫ�ģ�����û�ж���
   // int8_t drct=1;//���������������ת//2023/11/24���������Ҫ�䣬��������ʱ��Ҫ�ģ����Ǹ�����

 wheel_speed[0] = (sqrt(pow(vy_set + wz_set * Radius * 0.707107f, 2)
                     + pow(vx_set - wz_set * Radius * 0.707107f, 2)) * wheel_rpm_ratio);
wheel_speed[1] = (sqrt(pow(vy_set - wz_set * Radius * 0.707107f, 2)
                     + pow(vx_set - wz_set * Radius * 0.707107f, 2)) * wheel_rpm_ratio);
wheel_speed[2] = (sqrt(pow(vy_set - wz_set * Radius * 0.707107f, 2)
                     + pow(vx_set + wz_set * Radius * 0.707107f, 2)) * wheel_rpm_ratio);
wheel_speed[3] = (sqrt(pow(vy_set + wz_set * Radius * 0.707107f, 2)
                     + pow(vx_set + wz_set * Radius * 0.707107f, 2)) * wheel_rpm_ratio);

	  

}

/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fp32 chassis_6020_setangle[4]= {0.0f, 0.0f, 0.0f, 0.0f};	//�ĸ�6020����Ŀ��Ƕ�
    uint8_t i = 0;

    //mecanum wheel speed calculation
            //2023//11//24��Ϊ�����˶��ֽ⣬�����������Ը�Ϊ���֣����ֻ��ڣ��͸�һ���������־���
        chassis_vector_to_Steer_wheel_speed(chassis_move_control_loop->vx_set,
                                            chassis_move_control_loop->vy_set,
                                            chassis_move_control_loop->wz_set, wheel_speed);

    //���ֽǶȼ��� 
    AGV_angle_calc(chassis_move_control_loop->vx_set,
                                            chassis_move_control_loop->vy_set,
                                            chassis_move_control_loop->wz_set,chassis_6020_setangle );


    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
            chassis_move_control_loop->motor_steer[i].give_current = (int16_t)(chassis_6020_setangle[i]);
        }
        //in raw mode, derectly return
        //raw����ֱ�ӷ���
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //�������ӿ�������ٶȣ�������������ٶ�
        //2023//11//24�����ֿ��ܵ�����
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        chassis_move_control_loop->motor_steer[i].angle_set = chassis_6020_setangle[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //����pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
        chassis_move_control_loop->motor_chassis[i].speed,
        chassis_move_control_loop->motor_chassis[i].speed_set);



        
    }

    //���ʿ���
    // chassis_power_control(chassis_move_control_loop);


    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
    if(chassis_move_control_loop->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        if(chassis_move_control_loop->last_chassis_mode != CHASSIS_ZERO_FORCE)
        {
            //set chassis motor current to zero
            CAN_cmd_chassis(0, 0, 0, 0);
            // //set steer motor current to zero
             CAN_cmd_steer(0, 0, 0, 0);
        }
    }
    else
    {
        //send motor current to motor
        //���͵�����������
        CAN_cmd_chassis(chassis_move_control_loop->motor_chassis[0].give_current, 
                        chassis_move_control_loop->motor_chassis[1].give_current,
                        chassis_move_control_loop->motor_chassis[2].give_current, 
                        chassis_move_control_loop->motor_chassis[3].give_current);

        CAN_cmd_steer(chassis_move_control_loop->motor_chassis[0].give_current, 
                        chassis_move_control_loop->motor_chassis[1].give_current,
                        chassis_move_control_loop->motor_chassis[2].give_current, 
                        chassis_move_control_loop->motor_chassis[3].give_current);
    }
}
