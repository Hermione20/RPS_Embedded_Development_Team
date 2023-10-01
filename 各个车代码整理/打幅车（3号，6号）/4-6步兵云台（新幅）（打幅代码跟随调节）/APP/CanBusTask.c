#include "main.h"
#include "stdio.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};

volatile Encoder GM1Encoder1 = {0,0,0,0,0,0,0,0,0};
volatile Encoder GM2Encoder1 = {0,0,0,0,0,0,0,0,0};
volatile Encoder GM3Encoder1 = {0,0,0,0,0,0,0,0,0};
volatile Encoder GM4Encoder1 = {0,0,0,0,0,0,0,0,0};

volatile Encoder BUS1_CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder BUS1_CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder PokeEncoder = {0,0,0,0,0,0,0,0,0};




volatile capacitance_message1_t capacitance_message1 = {0};
volatile capacitance_message2_t capacitance_message2 = {0};
volatile distance_message_t distance_message = {0};
/*
***********************************************************************************************
*Name          :GetEncoderBias
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/


void DistanceProcess(volatile distance_message_t *v,CanRxMsg * msg)
{
	v->front_left_distance = (msg->Data[2]<<8)|msg->Data[3];
	v->front_right_distance = (msg->Data[4]<<8)|msg->Data[5];
	v->right_distance = (msg->Data[0]<<8)|msg->Data[1];

}

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{

            v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}

/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];
	v->torque_current=(msg->Data[4]<<8)|msg->Data[5];
}

void PitchEncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
//	v->last_raw_value = v->raw_value;
//	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
//	v->diff = v->raw_value - v->last_raw_value;
//	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
//	{
//		v->round_cnt++;
//		v->ecd_raw_rate = v->diff + 8192;
//	}
//	else if(v->diff>4096)
//	{
//		v->round_cnt--;
//		v->ecd_raw_rate = v->diff- 8192;
//	}		
//	else
//	{
//		v->ecd_raw_rate = v->diff;
//	}
//	v->ecd_value = v->raw_value + v->round_cnt * 8192;
//	//计算得到角度值，范围正负无穷大
//	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0439453125f  + v->round_cnt * 360;
//	v->filter_rate = ((msg->Data[2]<<8)|msg->Data[3])*0.0003;
////	if(v->filter_rate>32768)
////	{
////		v->filter_rate = (~((msg->Data[2]<<8)|msg->Data[3])+1);
////	}
//	v->temperature = msg->Data[6];
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];
}
/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/
u8  not_receive_time=0;


void CapacitanceProcess1(volatile capacitance_message1_t *v,CanRxMsg * msg)
{
float temp_sum=0; 
	v->charge_power    = msg->Data[1];
	v->charge_current  = msg->Data[2];
	v->raw_cap_voltage = msg->Data[3];
	v->boost_voltage   = msg->Data[4];
	v->battery_voltage = msg->Data[5];
	v->output_voltage	= msg->Data[6]; 
	v->cap_voltage_buff[v->buf_count++] = v->raw_cap_voltage;  //单位10v
	if(v->buf_count == VOLTAGE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算电容平均值
	for(int i = 0;i < VOLTAGE_BUF_SIZE; i++)
	{
		temp_sum += v->cap_voltage_buff[i];
	}
		v->cap_voltage_filte =(temp_sum/VOLTAGE_BUF_SIZE)/10.0;	
	not_receive_time=0;

}

void CapacitanceProcess2(volatile capacitance_message2_t *v,CanRxMsg * msg)
{
	v->max_charge_power   = msg->Data[1];
	v->max_charge_current = msg->Data[2];
	v->boost_voltage_ref  = msg->Data[3];
	v->output_mode_set    = msg->Data[4];
	v->output_mode_get    = msg->Data[5];
	
	
	v->fault_union.fault = msg->Data[6];
	
//	v->charge_over_current_flag  = (msg->Data[6]&0x01);
//	v->cap_over_voltage_flag     = (msg->Data[6]&0x02)>>1;
//	v->battery_over_under_voltage_flag = (msg->Data[6]&0x04)>>2;
//	v->battery_off_flag   = (msg->Data[6]&0x08)>>3;
//	v->two_leg_over_current_flag = (msg->Data[6]&0x10)>>4;
//	v->output_change_switch_flag = (msg->Data[6]&0x20)>>5;
//	v->boost_over_voltage_flag   = (msg->Data[6]&0x40)>>6;
//	v->boost_over_current_flag   = (msg->Data[6]&0x80)>>7;
	
	v->system_mode = msg->Data[7];
}


u32 can_receive_cnt = 0;
u32 can_last_receive_cnt = 0;
u8 yaw_error_flag=0;
u8 pitch_error_flag=0;
u8 shoot1_error_flag=0;
u8 shoot2_error_flag=0;
u8 poke_error_flag=0;

u8 abc[8] = {0};
float cap_voltage_filte;
void Can2ReceiveMsgProcess(CanRxMsg * msg)
{    yaw_error_flag=1;
     pitch_error_flag=1;  
        //GMYawEncoder.ecd_bias = yaw_ecd_bias;
        can2_count++;
		switch(msg->StdId)
		{
			
				case CAN_BUS2_DISTANCE_FEEDBACK_MSG_ID:
				{
					DistanceProcess(&distance_message,msg);       
                    
				}break;
				case CAN_BUS1_POWER_FEEDBACK_MSG_ID:
				{  
					cap_voltage_filte=(msg->Data[0]<<8)|msg->Data[1];
					capacitance_message1.cap_voltage_filte=cap_voltage_filte/10;
					
//					can_receive_cnt++;
//						not_receive_time=0;
//	        #if  NEW_CAP == 0
//					
//					capacitance_message2.fault_union.fault = (u8)(msg->Data[0]); 
//					capacitance_message2.system_mode = (u8)(msg->Data[1]);
//					capacitance_message1.cap_voltage = 0.1f*((int16_t)(msg->Data[2]<<8)|( int16_t)(msg->Data[3]));
//				  #elif NEW_CAP == 1
//					
//					if((u8)(msg->Data[0]) == 0x01)
//					{
//						CapacitanceProcess1(&capacitance_message1,msg);
//					}
//					else if((u8)(msg->Data[0]) == 0x02)
//					{
//						CapacitanceProcess2(&capacitance_message2,msg);
//					}
//				  #endif
//					
//					
				}break;
			
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
//					EncoderProcess(&GMYawEncoder ,msg);
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
					 //GMYawEncoder.ecd_bias = yaw_ecd_bias;
					 PitchEncoderProcess(&GMYawEncoder ,msg); 
           yaw_error_flag=0;					
						// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
					// if(can_count>=90 && can_count<=100)
					if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
					 {
							 if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000)
							 {
								GMYawEncoder.ecd_bias =GMYawEncoder_Offset + 8192;
							 }
							 else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
							 {
								GMYawEncoder.ecd_bias = GMYawEncoder_Offset - 8192;
							 }
					 }
				}break;
								case CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID:
				{
//					EncoderProcess(&GMYawEncoder ,msg);
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
					 //GMYawEncoder.ecd_bias = yaw_ecd_bias;
					 PitchEncoderProcess(&GMPitchEncoder ,msg);
           pitch_error_flag=0;			
						// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
					// if(can_count>=90 && can_count<=100)
					if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
					 {
							 if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
							 {
								GMPitchEncoder.ecd_bias =GMPitchEncoder_Offset + 8192;
							 }
							 else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
							 {
								GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset - 8192;
							 }
					 }
				}break;
			  case CAN_BUS2_THE_DOWN_CONTRAL_ID:
				{
					checkself.wheel_error_flag = (msg->Data[0]<<8)|msg->Data[1];
					
				}
				default:
				{
				}
				break;
		}	
		
		LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
//			if(!(fabs(pid_yaw.get), 70.0f) || GetWorkState() == STOP_STATE)  //如果是停止模式，一直喂狗防止重新启动失败
//			{
//				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
//			}		 
}
void Can1ReceiveMsgProcess(CanRxMsg * msg)
	{  
	 shoot1_error_flag=1;
   shoot2_error_flag=1;
	 poke_error_flag=1;
	 can1_count++;
	 	switch(msg->StdId)
		{
//				case CAN_BUS1_POWER_FEEDBACK_MSG_ID:
//				{
//					Fault = (u8)(msg->Data[0]); 
//					Sys_Mode=( u8)(msg->Data[1]);
//					V_Cap = 0.1f*((int16_t)(msg->Data[2]<<8)|( int16_t)(msg->Data[3]));
////					I_Icharge=0.1f*((int16_t)(msg->Data[4]<<8)|( int16_t)(msg->Data[5]));
//					Vin=0.1f*((int16_t)(msg->Data[6]<<8)|( int16_t)(msg->Data[7]));
//				}break;
			case CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID:
				{
					
					shoot1_error_flag=0;
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
					(can1_count<=50) ? GetEncoderBias(&BUS1_CM1Encoder ,msg):EncoderProcess(&BUS1_CM1Encoder,msg);       //获取到编码器的初始偏差值            
                    
				}break;
				case CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID:
				{
					
					shoot2_error_flag=0;
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
					(can1_count<=50) ? GetEncoderBias(&BUS1_CM2Encoder,msg):EncoderProcess(&BUS1_CM2Encoder ,msg);
				}break;
	      
	      case CAN_BUS1_POKE_FEEDBACK_MSG_ID:
				{
					
					poke_error_flag=0;
//					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
					(can1_count<=50) ? GetEncoderBias(&PokeEncoder,msg):EncoderProcess(&PokeEncoder ,msg);
				}break;	
				default:
				{
				}
				break;
	    
		}
	}

/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/


/********************************************************************************
   给电调板发送指令，ID号为0x1FF，只用两个电调板，数据回传ID为0x205和0x206
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}

