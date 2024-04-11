#ifndef __BUFF_KARMAN_FILTER_H
#define __BUFF_KARMAN_FILTER_H
#include "public.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_copy    arm_copy_f32
#define dt 0.001
typedef struct
{
	float buff_yaw_angle;
	float buff_pitch_angle;
	float buff_yaw_speed;
	float buff_pitch_speed;
	
	mat b_xhat,
	    b_A,
	    b_AT,
	    b_H,
	    b_HT,
	    b_Z,
	    b_R,
	    b_Q,
	    b_P,
	    b_K,
	    b_I;
	
	float xhat_data[4];
	float	z_data[2];					 
	float	K_data[8];
	float P_data[16];
	float AT_data[16];
	float A_data[16];
	float HT_data[8];
	float H_data[8];
	float Q_data[16];
	float R_data[4];
	float I_data[16];
}buff_kalman_filter_t;

void buff_karman_filter_Init(buff_kalman_filter_t *B);
void buff_karman_filter_calc(buff_kalman_filter_t *B,float ecd_yaw_angle,float ecd_pitch_angle,u8 *buff_kf_flag);
void buff_kalman_filter_reset(buff_kalman_filter_t *B);

extern buff_kalman_filter_t buff_kalman_filter;
extern float buff_kf_flag;
#endif
