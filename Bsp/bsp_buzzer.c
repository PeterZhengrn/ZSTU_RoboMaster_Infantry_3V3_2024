#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
/**
 * @brief  ��������ʼ��
 * @param  None
 * @retval None
 */
void buzzer_init(void)
{
	//start tim
	//������ʱ��
	HAL_TIM_Base_Start(&htim4);
	//start pwm channel
	//����PWMͨ��
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
/**
 * @brief  ��������
 * @param  psc: Ԥ��Ƶֵ
 * @param  pwm: PWMֵ
 * @retval None
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
/**
 * @brief  ��������
 * @param  None
 * @retval None
 */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
/**
 * @brief  ����������
 * @param  None
 * @retval None
 */
void buzzer_high(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 1);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
/**
 * @brief  ����������
 * @param  None
 * @retval None
 */
void buzzer_mid(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 7);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
/**
 * @brief  ����������
 * @param  None
 * @retval None
 */
void buzzer_low(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 20);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
