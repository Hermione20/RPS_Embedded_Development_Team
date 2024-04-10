#include "buff_karman_filter.h"
float buff_kf_flag = 0;

buff_kalman_filter_t buff_kalman_filter =
{
	.Q_data = {
	            20 , 0 , 0 , 0,
	            0 , 20 , 0 , 0,
	            0 , 0 , 70 , 0,
	            0 , 0 , 0 , 70
            },
	.R_data = {
		          0.1 , 0,
		          0 , 0.1
	          },
	.A_data = {
		         1 , 0 , dt , 0,
		         0 , 1 , 0 , dt,
		         0 , 0 , 1 , 0,
		         0 , 0 , 0 , 1
	         },
	.H_data = {
		         1 , 0 , 0 , 0, 
		         0 , 1 , 0 , 0		
	          },
	.I_data = {
	            1 , 0 , 0 , 0,
	            0 , 1 , 0 , 0,
	            0 , 0 , 1 , 0,
	            0 , 0 , 0 , 1
            },
};

void buff_karman_filter_Init(buff_kalman_filter_t *B)
{
	mat_init(&B->b_xhat,4,1,(float *)B->xhat_data);
	mat_init(&B->b_Z,2,1, (float *)B->z_data);
	mat_init(&B->b_A,4,4, (float *)B->A_data);
	mat_init(&B->b_AT,4,4,(float *)B->AT_data);
	mat_trans(&B->b_A,&B->b_AT);
	mat_init(&B->b_Q,4,4,(float *)B->Q_data);
	mat_init(&B->b_H,2,4,(float *)B->H_data);
	mat_init(&B->b_HT,4,2,(float *)B->HT_data);
	mat_trans(&B->b_H,&B->b_HT);
	mat_init(&B->b_R,2,2,(float *)B->R_data);
	mat_init(&B->b_P,4,4,(float *)B->P_data);
	mat_init(&B->b_K,4,2,(float *)B->K_data);
	mat_init(&B->b_I,4,4,(float *)B->I_data);
}
	float TEMP41_data[4] = {0};float bTEMP2_data[16] = {0};float TEMP24_data[8] = {0};float TEMP_22_data[4] = {0};float TEMP__2_data[2] = {0};
	float bTEMP_data[16] = {0};float TEMP42_data[8] = {0};float TEMP22_data[4] = {0};float TEMP_2_data[2] = {0};float TEMP_41_data[4] = {0};
void buff_karman_filter_calc(buff_kalman_filter_t *B,float ecd_yaw_angle,float ecd_pitch_angle,u8 *buff_kf_flag)
{
	static u8 state_kalman_init = 0;
	if(state_kalman_init == 0)
	{
		state_kalman_init = 1;
		buff_karman_filter_Init(&buff_kalman_filter);
		
	}
	
	mat TEMP;
	mat_init(&TEMP,4,4,(float *)bTEMP_data);
	
	
	mat TEMP2;
	mat_init(&TEMP2,4,4,(float *)bTEMP2_data);
	
	
	mat TEMP42;
	mat_init(&TEMP42,4,2,(float *)TEMP42_data);
	
	
	mat TEMP24;
	mat_init(&TEMP24,2,4,(float *)TEMP24_data);
	
	
	mat TEMP22;
	mat_init(&TEMP22,2,2,(float *)TEMP22_data);
	
  
	mat TEMP_22;
	mat_init(&TEMP_22,2,2,(float *)TEMP_22_data);
	
	
	mat TEMP_2;
	mat_init(&TEMP_2,2,1,(float *)TEMP_2_data);
	
	
	mat TEMP__2;
	mat_init(&TEMP__2,2,1,(float *)TEMP__2_data);
	

	mat TEMP41;
	mat_init(&TEMP41,4,1,(float *)TEMP41_data);
	
	
	mat TEMP_41;
	mat_init(&TEMP_41,4,1,(float *)TEMP_41_data);
	
	B->b_Z.pData[0] = ecd_yaw_angle;
	B->b_Z.pData[1] = ecd_pitch_angle;
	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&B->b_A, &B->b_xhat,&TEMP41);
	mat_copy(TEMP41_data,B->xhat_data,4);
	if(*buff_kf_flag)
	{
	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&B->b_A, &B->b_P,&TEMP);
	mat_mult(&TEMP,&B->b_AT,&TEMP2);
	mat_add(&TEMP2,&B->b_Q,&B->b_P);
	
	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&B->b_P,&B->b_HT,&TEMP42);
	
	mat_mult(&B->b_H,&B->b_P,&TEMP24);
	mat_mult(&TEMP24,&B->b_HT,&TEMP22);
	mat_add(&TEMP22,&B->b_R,&TEMP_22);
	mat_inv(&TEMP_22,&TEMP22);
	
	mat_mult(&TEMP42,&TEMP22,&B->b_K);
	
	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&B->b_H,&B->b_xhat,&TEMP_2);
	mat_sub(&B->b_Z,&TEMP_2,&TEMP__2);
	mat_mult(&B->b_K,&TEMP__2,&TEMP41);
	mat_add(&B->b_xhat,&TEMP41,&TEMP_41);
	mat_copy(TEMP_41_data,B->xhat_data,4);
	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&B->b_K,&B->b_H,&TEMP);
	mat_sub(&B->b_I,&TEMP,&TEMP2);
	mat_mult(&TEMP2,&B->b_P,&TEMP);
	mat_copy(bTEMP_data,B->P_data,16);
	
	*buff_kf_flag = 0;
}
	
	B->buff_yaw_angle = B->b_xhat.pData[0];
	B->buff_pitch_angle = B->b_xhat.pData[1];
	B->buff_yaw_speed = B->b_xhat.pData[2];
	B->buff_pitch_speed = B->b_xhat.pData[3];
}

void buff_kalman_filter_reset(buff_kalman_filter_t *B)
{
	buff_karman_filter_Init(&buff_kalman_filter);
	B->b_xhat.pData[0] = 0;
	B->b_xhat.pData[1] = 0;
	B->b_xhat.pData[2] = 0;
	B->b_xhat.pData[3] = 0;
}

