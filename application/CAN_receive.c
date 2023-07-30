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

#include "CAN_receive.h"
#include "main.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//???��?????��??
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//???��?��???��??
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//???����????????��??
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
static void get_gimbal_measure(CANx_t *gimbal_measure,uint8_t data[8]);
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
motor_measure_t motor_chassis[7];

CANx_t gimbal[2];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
/**
 * @brief  ������λ���ٶ�ģʽ�¿���֡
 * @param  hcan   CAN�ľ��
 * @param  motor  ����ṹ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef*hcan, CANx_t *motor ,uint16_t id,float _pos,float _vel);
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
        case DM_YAW_MOTOR_ID:
        case DM_PIT_MOTOR_ID:
        {
          static uint8_t i = 0;
          i = rx_header.StdId - DM_YAW_MOTOR_ID;
          get_gimbal_measure(&gimbal[i],rx_data);
          break;
        }
        default:
        {
            break;
        }
    }
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//����CAN�ж�֪ͨ
}
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
static void get_gimbal_measure(CANx_t *gimbal_measure,uint8_t data[8])
{
  gimbal_measure->p_int=(int)(data[1]<<8)|data[2];
  gimbal_measure->v_int=(int)(data[3]<<8)|(data[4]>>4);
  gimbal_measure->t_int=(int)((data[4]&0xF)<<8)|data[5];
  gimbal_measure->position=uint_to_float(gimbal_measure->p_int,P_MIN,P_MAX,16);// (-12.5,12.5)
  gimbal_measure->velocity=uint_to_float(gimbal_measure->v_int,V_MIN,V_MAX,12);// (-45.0,45.0)
  gimbal_measure->torque=uint_to_float(gimbal_measure->t_int,T_MIN,T_MAX,12);// (-18.0,18.0)
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
  /*�ҵ��յķ������䣬�����ݷ��ͳ�ȥ*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}
/**
 * @brief  �����IDΪ0x02��0x03��0x04��3�������������ʹ�ܣ��ڶԵ�����п���ǰ����ʹ��
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */
void Motor_enable(void)
{
//	#if Motar_mode==0
//	CANx_SendStdData(&hcan1,0x02,Data_Enable,8);	
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x03,Data_Enable,8);
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x04,Data_Enable,8);
//	HAL_Delay(10);
//	#elif Motar_mode==1
//	CANx_SendStdData(&hcan1,0x102,Data_Enable,8);	
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x103,Data_Enable,8);
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x104,Data_Enable,8);
//	HAL_Delay(10);
//	#elif Motar_mode==2
//	CANx_SendStdData(&hcan1,0x202,Data_Enable,8);	
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x203,Data_Enable,8);
//	HAL_Delay(10);
//	CANx_SendStdData(&hcan1,0x204,Data_Enable,8);
//	HAL_Delay(10);
//	#endif
//		CANx_SendStdData(&GIMBAL_CAN,0x105,Data_Enable,8);	
//		HAL_Delay(10);
//		CANx_SendStdData(&GIMBAL_CAN,0x106,Data_Enable,8);
//		HAL_Delay(10);
		CANx_SendStdData(&GIMBAL_CAN,0x105,Data_Enable,8);	
		HAL_Delay(10);
		CANx_SendStdData(&GIMBAL_CAN,0x106,Data_Enable,8);
		HAL_Delay(10);
}

/**
 * @brief  ������λ���ٶ�ģʽ�¿���֡
 * @param  hcan   CAN�ľ��
 * @param  motor  ����ṹ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef*hcan, CANx_t *motor ,uint16_t id,float _pos,float _vel)
{
    static CAN_TxHeaderTypeDef Tx_Header;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		Tx_Header.StdId=id;
		Tx_Header.IDE=CAN_ID_STD;
		Tx_Header.RTR=CAN_RTR_DATA;
		Tx_Header.DLC=0x08;

    motor->Tx_Data[0] = *pbuf;;
    motor->Tx_Data[1] = *(pbuf+1);
    motor->Tx_Data[2] = *(pbuf+2);
    motor->Tx_Data[3] = *(pbuf+3);
    motor->Tx_Data[4] = *vbuf;
    motor->Tx_Data[5] = *(vbuf+1);
    motor->Tx_Data[6] = *(vbuf+2);
    motor->Tx_Data[7] = *(vbuf+3);
    //�ҵ��յķ������䣬���ʼ�����ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, motor->Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
      if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, motor->Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
      {
        HAL_CAN_AddTxMessage(hcan, &Tx_Header, motor->Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
      }
    }
}
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
