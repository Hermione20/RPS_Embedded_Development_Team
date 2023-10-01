#include "main.h"
float autoshoot_kf_flag = 0;

autoshoot_kalman_filter_t autoshoot_kalman_filter =
{
	.Q_data = {
	            1 , 0 , 0 , 0 , 0,
	            0 , 1 , 0 , 0 , 0,
	            0 , 0 , 9 , 0 , 0,
	            0 , 0 , 0 , 0.1 , 0,
		          0 , 0 , 0 , 0 , 50
            },
	.R_data = {
		          20 , 0 , 0,
		          0 , 10 , 0,
		          0 , 0 , 10
	          },
	.A_data = {
		         1 , 0 , Dt , 0 , Dt*Dt/2.0f,
		         0 , 1 , 0 , Dt , 0,
		         0 , 0 , 1 , 0  , Dt,
		         0 , 0 , 0 , 1  , 0,
		         0 , 0 , 0 , 0  , 1
	         },
	.H_data = {
		         1 , 0 , 0 , 0 , 0,
		         0 , 1 , 0 , 0 , 0,
		         0 , 0 , 1 , 0 , 0,
	          },
	.I_data = {
	            1 , 0 , 0 , 0 , 0,
	            0 , 1 , 0 , 0 , 0,
	            0 , 0 , 1 , 0 , 0,
	            0 , 0 , 0 , 1 , 0,
		          0 , 0 , 0 , 0 , 1,
            },
};

void autoshoot_karman_filter_Init(autoshoot_kalman_filter_t *B)
{
	mat_init(&B->b_xhat,5,1,(float *)B->xhat_data);
	mat_init(&B->b_Z,3,1, (float *)B->z_data);
	mat_init(&B->b_A,5,5, (float *)B->A_data);
	mat_init(&B->b_AT,5,5,(float *)B->AT_data);
	mat_trans(&B->b_A,&B->b_AT);
	mat_init(&B->b_Q,5,5,(float *)B->Q_data);
	mat_init(&B->b_H,3,5,(float *)B->H_data);
	mat_init(&B->b_HT,5,3,(float *)B->HT_data);
	mat_trans(&B->b_H,&B->b_HT);
	mat_init(&B->b_R,3,3,(float *)B->R_data);
	mat_init(&B->b_P,5,5,(float *)B->P_data);
	mat_init(&B->b_K,5,3,(float *)B->K_data);
	mat_init(&B->b_I,5,5,(float *)B->I_data);
	
}
float TEMP51_data[5] = {0};float TEMP_data[25] = {0};float TEMP2_data[25] = {0};float TEMP53_data[15] = {0};float TEMP35_data[15] = {0};float TEMP33_data[9] = {0};
float TEMP_33_data[9] = {0};float TEMP_3_data[3] = {0};float TEMP__3_data[3] = {0};float TEMP_51_data[5] = {0};
void autoshoot_karman_filter_calc(autoshoot_kalman_filter_t *B,float ecd_yaw_angle,float ecd_pitch_angle,float ecd_yaw_speed)
{
	static u8 state_kalman_init = 0;
	if(state_kalman_init == 0)
	{
		state_kalman_init = 1;
		autoshoot_karman_filter_Init(&autoshoot_kalman_filter);
		
	}
	
	
	mat TEMP51;
	mat_init(&TEMP51,5,1,(float *)TEMP51_data);
	
	
	mat TEMP;
	mat_init(&TEMP,5,5,(float *)TEMP_data);
	
	
	mat TEMP2;
	mat_init(&TEMP2,5,5,(float *)TEMP2_data);
	
	
	mat TEMP53;
	mat_init(&TEMP53,5,3,(float *)TEMP53_data);
	
	
	mat TEMP35;
	mat_init(&TEMP35,3,5,(float *)TEMP35_data);
	
	
	mat TEMP33;
	mat_init(&TEMP33,3,3,(float *)TEMP33_data);
	
  
	mat TEMP_33;
	mat_init(&TEMP_33,3,3,(float *)TEMP_33_data);
	
	
	mat TEMP_3;
	mat_init(&TEMP_3,3,1,(float *)TEMP_3_data);
	
	
	mat TEMP__3;
	mat_init(&TEMP__3,3,1,(float *)TEMP__3_data);
	
	
	mat TEMP_51;
	mat_init(&TEMP_51,5,1,(float *)TEMP_51_data);
	
	B->b_Z.pData[0] = ecd_yaw_angle;
	B->b_Z.pData[1] = ecd_pitch_angle;
	B->b_Z.pData[2] = ecd_yaw_speed;
	
	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&B->b_A, &B->b_xhat,&TEMP51);
	mat_copy(TEMP51_data,B->xhat_data,5);
	
	if((abs(B->b_Z.pData[0] - B->b_xhat.pData[0])>=15)|(abs(B->b_Z.pData[1] - B->b_xhat.pData[1])>=10))
		autoshoot_kalman_filter_reset(&autoshoot_kalman_filter);

	if(autoshoot_kf_flag)
	{
	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&B->b_A, &B->b_P,&TEMP);
	mat_mult(&TEMP,&B->b_AT,&TEMP2);
	mat_add(&TEMP2,&B->b_Q,&B->b_P);
	
	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&B->b_P,&B->b_HT,&TEMP53);
	
	mat_mult(&B->b_H,&B->b_P,&TEMP35);
	mat_mult(&TEMP35,&B->b_HT,&TEMP33);
	mat_add(&TEMP33,&B->b_R,&TEMP_33);
	mat_inv(&TEMP_33,&TEMP33);
	
	mat_mult(&TEMP53,&TEMP33,&B->b_K);
	
	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&B->b_H,&B->b_xhat,&TEMP_3);
	mat_sub(&B->b_Z,&TEMP_3,&TEMP__3);
	mat_mult(&B->b_K,&TEMP__3,&TEMP51);
	mat_add(&B->b_xhat,&TEMP51,&TEMP_51);
	mat_copy(TEMP_51_data,B->xhat_data,5);
	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&B->b_K,&B->b_H,&TEMP);
	mat_sub(&B->b_I,&TEMP,&TEMP2);
	mat_mult(&TEMP2,&B->b_P,&TEMP);
	mat_copy(TEMP_data,B->P_data,25);
	
	autoshoot_kf_flag = 0;
}
	
	
	B->auto_yaw_angle = B->b_xhat.pData[0];
	B->auto_pitch_angle = B->b_xhat.pData[1];
	B->auto_yaw_speed = B->b_xhat.pData[2];
	B->auto_pitch_speed = B->b_xhat.pData[3];
  B->auto_yaw_acc = B->b_xhat.pData[4];
}

void autoshoot_kalman_filter_reset(autoshoot_kalman_filter_t *B)
{
	
	autoshoot_karman_filter_Init(&autoshoot_kalman_filter);
	B->b_xhat.pData[0] = yaw_Angle;
	B->b_xhat.pData[1] = pitch_Angle;
	B->b_xhat.pData[2] = 0;
	B->b_xhat.pData[3] = 0;
	B->b_xhat.pData[4] = 0;
	B->b_Z.pData[0] = yaw_Angle;
	B->b_Z.pData[1] = pitch_Angle;
	B->b_Z.pData[2] = 0;
//	Set_Gimbal_Current(CAN2, 0, 0);
	
}

