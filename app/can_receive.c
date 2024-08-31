#include "can_receive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

	
static motor_measure_t motor_chassis[7];

static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t chassis_can_send_data[8];	

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//中断回调函数 接收电机反馈报文
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
	
	switch(rx_header.StdId)
	{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		case CAN_YAW_MOTOR_ID:
		case CAN_PIT_MOTOR_ID:
		case CAN_TRIGGER_MOTOR_ID:
		{
			static uint8_t i=0;
			i=rx_header.StdId-CAN_3508_M1_ID;
			get_motor_measure(&motor_chassis[i], rx_data);
			break;
		}
		default:
		{
			break;
		}
	}

}

void CAN_cmd_gimbal(int16_t yaw,int16_t pitch,int16_t shoot,int16_t rev)//云台报文发送
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId=CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE=CAN_ID_STD;
	gimbal_tx_message.RTR=CAN_RTR_DATA;
	gimbal_tx_message.DLC=0x08;
	
	gimbal_can_send_data[0]=(yaw >> 8);
	gimbal_can_send_data[1]=yaw;
	gimbal_can_send_data[2]=(pitch>>8);
	gimbal_can_send_data[3]=pitch;
	gimbal_can_send_data[4]=(shoot>>8);
	gimbal_can_send_data[5]=shoot;
	gimbal_can_send_data[6]=(rev>>8);
	gimbal_can_send_data[7]=rev;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN,&gimbal_tx_message,gimbal_can_send_data,&send_mail_box);
	
}
	
void CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)//底盘报文发送
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId=CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE=CAN_ID_STD;
	chassis_tx_message.RTR=CAN_RTR_DATA;
	chassis_tx_message.DLC=0x08;
	
	chassis_can_send_data[0]=(motor1 >> 8);
	chassis_can_send_data[1]=motor1;
	chassis_can_send_data[2]=(motor2>>8);
	chassis_can_send_data[3]=motor2;
	chassis_can_send_data[4]=(motor3>>8);
	chassis_can_send_data[5]=motor3;
	chassis_can_send_data[6]=(motor4>>8);
	chassis_can_send_data[7]=motor4;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN,&chassis_tx_message,chassis_can_send_data,&send_mail_box);
	
}

const motor_measure_t *get_chassis_motor_measure(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];//后两位是电机id号
}
