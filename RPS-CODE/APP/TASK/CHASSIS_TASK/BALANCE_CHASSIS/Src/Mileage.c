#include "Mileage.h"

/**
  ******************************************************************************
  * @file    Mileage.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    28-November-2023
  * @brief   平衡步兵用里程计，该里程计使用卡尔曼滤波器将imu加速度计
							和轮式编码器进行融合，仅融合一个方向的速度和位置，用于
							平步的打滑检测。
						 
@verbatim
 ===============================================================================
 **/
 
 //数值初始化
Mileage_kalman_filter_t Mileage_kalman_filter =
{
	.Q_data = {
	            0.1 , 0 ,
	            0 , 0.1 
            },
	.R_data = {
		          1 , 0,
		          0 , 1000
	          },
	.A_data = {
		         1 , DT ,
		         0 , 1 
						},
	.B_data = {
							0.5*DT*DT,
							DT
						},
	.H_data = {
		         1 , 0 , 
		         0 , 1	
	          },
	.I_data = {
	            1 , 0 ,
	            0 , 1 	            
            },
};


//滤波器初始化
void Mileage_karman_filter_Init(Mileage_kalman_filter_t *B)
{
	mat_init(&B->xhat,2,1,(float *)B->xhat_data);
	mat_init(&B->Z,2,1, (float *)B->z_data);
	mat_init(&B->A,2,2, (float *)B->A_data);
	mat_init(&B->AT,2,2,(float *)B->AT_data);
	mat_trans(&B->A,&B->AT);
	mat_init(&B->B,2,1, (float *)B->B_data);
	mat_init(&B->U,1,1, (float *)B->U_data);
	mat_init(&B->Q,2,2,(float *)B->Q_data);
	mat_init(&B->H,2,2,(float *)B->H_data);
	mat_init(&B->HT,2,2,(float *)B->HT_data);
	mat_trans(&B->H,&B->HT);
	mat_init(&B->R,2,2,(float *)B->R_data);
	mat_init(&B->P,2,2,(float *)B->P_data);
	mat_init(&B->K,2,2,(float *)B->K_data);
	mat_init(&B->I,2,2,(float *)B->I_data);
}


//以下为滤波器的中间变量，无实际含义，仅计算用
static float TEMP21_data[2] = {0};
static float TEMP21__data[2] = {0};
static float TEMP21___data[2] = {0};
static float TEMP22_data[4] = {0};
static float TEMP22__data[4] = {0};
static float TEMP22___data[4] = {0};
static float TEMP22____data[4] = {0};


void Mileage_kalman_filter_calc(Mileage_kalman_filter_t *B,float ecd_position,float ecd_velocity,float acc)
{
	static u8 state_kalman_init = 0;
	if(state_kalman_init == 0)
	{
		state_kalman_init = 1;
		Mileage_karman_filter_Init(B);
		
	}
	
	mat TEMP21;
	mat_init(&TEMP21,2,1,(float *)TEMP21_data);
	
	mat TEMP21_;
	mat_init(&TEMP21_,2,1,(float *)TEMP21__data);
	
	mat TEMP21__;
	mat_init(&TEMP21__,2,1,(float *)TEMP21___data);
	
	
	mat TEMP22;
	mat_init(&TEMP22,2,2,(float *)TEMP22_data);
	
	mat TEMP22_;
	mat_init(&TEMP22_,2,2,(float *)TEMP22__data);
	
	mat TEMP22__;
	mat_init(&TEMP22__,2,2,(float *)TEMP22___data);	
	
	mat TEMP22___;
	mat_init(&TEMP22___,2,2,(float *)TEMP22____data);	
	
	B->Z.pData[0] = ecd_position;
	B->Z.pData[1] = ecd_velocity;
	B->U.pData[0] = acc;
	
	//1. xhat'(k)= A xhat(k-1) + B u(k-1)
	mat_mult(&B->A, &B->xhat,&TEMP21);
	mat_mult(&B->B, &B->U,&TEMP21_);
	mat_add(&TEMP21,&TEMP21_,&TEMP21__);
	mat_copy(TEMP21___data,B->xhat_data,2);
	
	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&B->A, &B->P,&TEMP22);
	mat_mult(&TEMP22,&B->AT,&TEMP22_);
	mat_add(&TEMP22_,&B->Q,&B->P);
	
	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&B->P,&B->HT,&TEMP22);
	
	mat_mult(&B->H,&B->P,&TEMP22_);
	mat_mult(&TEMP22_,&B->HT,&TEMP22__);
	mat_add(&TEMP22__,&B->R,&TEMP22_);
	mat_inv(&TEMP22_,&TEMP22___);
	
	mat_mult(&TEMP22,&TEMP22___,&B->K);
	
	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&B->H,&B->xhat,&TEMP21);
	mat_sub(&B->Z,&TEMP21,&TEMP21_);
	mat_mult(&B->K,&TEMP21_,&TEMP21);
	mat_add(&B->xhat,&TEMP21,&TEMP21_);
	mat_copy(TEMP21__data,B->xhat_data,2);
	
	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&B->K,&B->H,&TEMP22);
	mat_sub(&B->I,&TEMP22,&TEMP22_);
	mat_mult(&TEMP22_,&B->P,&TEMP22);
	mat_copy(TEMP22_data,B->P_data,4);
	
	B->positon = B->xhat.pData[0];
	B->velocity = B->xhat.pData[1];
}

//滤波器的重置
void Mileage_kalman_filter_reset(Mileage_kalman_filter_t *B)
{
	Mileage_karman_filter_Init(&Mileage_kalman_filter);
	B->xhat.pData[0] = 0;
	B->xhat.pData[1] = 0;
}
