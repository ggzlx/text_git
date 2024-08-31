#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H
#include "struct.h"
#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID =0x202,
	CAN_3508_M3_ID =0x203,
	CAN_3508_M4_ID =0x204,
	
	CAN_YAW_MOTOR_ID =0x205,
	CAN_PIT_MOTOR_ID =0x206,
	CAN_TRIGGER_MOTOR_ID =0x207,
	CAN_GIMBAL_ALL_ID =0x1FF,
	
}can_msg_id_e;
extern void CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
#endif

