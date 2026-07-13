#include "Can_Feedback.h"

/*
     Forward
    0       1 (3508)
				0 (YAW 6020)
						1 (PITCH 6020)
*/

//INCLUDE部分
#include "Gimbal_Task.h"
//全局变量定义部分
Gimbal_Motor_t Gimbal_Motor;
Down_Cboard_Info_t Down_Cboard_Info;

/*******************************************************************************************************
接收电机的反馈值并处理(CAN1)
********************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t Rx_Data1[8];
	static CAN_RxHeaderTypeDef CAN1_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN1_RxHeader,Rx_Data1);
	
	while(RxState!=HAL_OK){;}

	switch(CAN1_RxHeader.StdId)
	{
		case 0x104:
		{
//			Down_Cboard_Info.fall_flag  = 0;
			Down_Cboard_Info.fall_flag  = Rx_Data1[0]&0x01;
			Down_Cboard_Info.enem_color = Rx_Data1[0]&0x02;
			
			Down_Cboard_Info.heat_limit    = (int16_t)(Rx_Data1[1] << 8 | Rx_Data1[2]);
			Down_Cboard_Info.barrel_heat   = (int16_t)(Rx_Data1[3] << 8 | Rx_Data1[4]);
			Down_Cboard_Info.cooling_value =  Rx_Data1[5];
			
			Down_Cboard_Info.down_dyaw     = (float)((int16_t)(Rx_Data1[6] << 8 | Rx_Data1[7]))/1000.f;
			return;
		}
		default:
		{
			break;
		}
	}
}

/*******************************************************************************************************
接收电机的反馈值并处理(CAN2)
********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t Rx_Data2[8]; 
  static CAN_RxHeaderTypeDef CAN2_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN2_RxHeader,Rx_Data2);

	while(RxState!=HAL_OK){;}
		
	switch(CAN2_RxHeader.StdId)
	{
		case 0x205://YAW
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_6020[0],Rx_Data2);
			break;
		}
		case 0x206://PITCH
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_6020[1],Rx_Data2);
			break;
		}
		case 0x201://摩擦轮右
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_3508[1],Rx_Data2);
			break;
		}
		case 0x202://摩擦轮左
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_3508[0],Rx_Data2);
			break;
		}
		case 0x000://拨盘
		{
			Receive_Motor_Data(&Gimbal_Motor.DM_4310,Rx_Data2,DM4310);
			break;
		}
		default:
		{
			break;
		}
	}
}


