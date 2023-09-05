#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/**
 * @brief  CAN��������ʼ�� 
 * @param  None
*/
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;//CAN�˲����ṹ��
    can_filter_st.FilterActivation = ENABLE;//ʹ���˲���
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//ID����ģʽ
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32λ�˲���
    can_filter_st.FilterIdHigh = 0x0000;//32λID
    can_filter_st.FilterIdLow = 0x0000;//32λID
    can_filter_st.FilterMaskIdHigh = 0x0000;//32λ����ID
    can_filter_st.FilterMaskIdLow = 0x0000;//32λ����ID
    can_filter_st.FilterBank = 0;//��������0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//��������0������FIFO0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//���ù�����
    HAL_CAN_Start(&hcan1);//����CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//�����ж�


    can_filter_st.SlaveStartFilterBank = 14;//�ӹ�������14��ʼ
    can_filter_st.FilterBank = 14;//��������14
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);//���ù�����
    HAL_CAN_Start(&hcan2);//����CAN2
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//�����ж�



}
