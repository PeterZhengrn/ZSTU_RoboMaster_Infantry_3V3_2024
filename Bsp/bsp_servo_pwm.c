#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;


/**
 * @brief  �ŷ����PWM���
 * @param  pwm: PWMֵ ��Χ: 0 ~ 19999 ռ�ձȣ�0 ~ 100
 * @param  i: �ŷ������� ��Χ��0 ~ 6
 * @retval None
*/
void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
        }break;
        case 1:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);
        }break;
        case 2:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
        }break;
        case 3:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
        }break;
				case 4:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
        }break;
        case 5:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
        }break;
        case 6:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
        }break;
    }
}


/**
 * @brief  �ŷ����PWM��ʼ��
 * @param  None
 * @retval None
*/
void servo_pwm_init(void)
{
	  HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}
