#ifndef __AUTOSHOOT_KARMAN_FILTER_H
#define __AUTOSHOOT_KARMAN_FILTER_H
#include "main.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_copy    arm_copy_f32

#define Dt 0.001f

typedef struct
{
	float auto_yaw_angle;
	float auto_pitch_angle;
	float auto_yaw_speed;
	float auto_pitch_speed;
	float auto_yaw_acc;
	
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
	
	float xhat_data[5];
	float	z_data[3];					 
	float	K_data[15];
	float P_data[25];
	float AT_data[25];
	float A_data[25];
	float HT_data[15];
	float H_data[15];
	float Q_data[25];
	float R_data[9];
	float I_data[25];
}autoshoot_kalman_filter_t;

void autoshoot_karman_filter_Init(autoshoot_kalman_filter_t *B);
void autoshoot_karman_filter_calc(autoshoot_kalman_filter_t *B,float ecd_yaw_angle,float ecd_pitch_angle,float ecd_yaw_speed);
void autoshoot_kalman_filter_reset(autoshoot_kalman_filter_t *B);

extern float autoshoot_kf_flag;
extern autoshoot_kalman_filter_t autoshoot_kalman_filter;

#endif

