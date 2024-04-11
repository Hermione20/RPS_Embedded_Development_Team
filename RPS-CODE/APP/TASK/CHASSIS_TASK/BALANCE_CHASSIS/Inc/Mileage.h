#ifndef __MILEAGE_H
#define __MILEAGE_H
#include "public.h"




#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_copy    arm_copy_f32
#define DT 0.001



typedef struct
{
	float positon;
	float velocity;

	mat xhat,
			U,
	    A,
	    AT,
			B,
	    H,
	    HT,
	    Z,
	    R,
	    Q,
	    P,
	    K,
	    I;
	
	float xhat_data[2];
	float U_data[1];
	float B_data[2];
	float	z_data[2];					 
	float	K_data[4];
	float P_data[4];
	float AT_data[4];
	float A_data[4];
	float HT_data[4];
	float H_data[4];
	float Q_data[4];
	float R_data[4];
	float I_data[4];
}Mileage_kalman_filter_t;





void Mileage_karman_filter_Init(Mileage_kalman_filter_t *B);
void Mileage_kalman_filter_calc(Mileage_kalman_filter_t *B,float ecd_position,float ecd_velocity,float acc);
void Mileage_kalman_filter_reset(Mileage_kalman_filter_t *B);



extern Mileage_kalman_filter_t Mileage_kalman_filter;







#endif
