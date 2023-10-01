/* second-order kalman filter on stm32 */
#include "kalman_filter.h"

kalman_filter_init_t kalman_filter_I = 
{	.Q_data = {1,0,0,0, 				0,1,0,0, 				 0,0,1,0, 		0,0,0,1		}, 	//过程噪声激励协方差矩阵
	.R_data = {0.5,0,0,0, 			0,0.2,0,0, 		 0,0,50,0, 	0,0,0,50},	//测量误差矩阵,根据实际测量误差进行优化，值越小输出结果依赖观测的量越强//0.2 0.2 50 50
	.A_data = {1,0,Ts_Filter,0, 0,1,0,Ts_Filter, 0,0,1,0, 		0,0,0,1		}, 	//状态转移矩阵 					
	.H_data = {1,0,0,0, 				0,1,0,0, 				 0,0,1,0, 		0,0,0,1		}};	//观测矩阵,观测了位置和速度，因此设置为单位矩阵。
																																					//如果没有观测速度则后两列均为0，会导致输出延迟
kalman_filter_t kalman_filter_F;



void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,4,1,(float *)I->xhat_data);						
//  ...
	mat_init(&F->xhatminus,4,1,(float *)I->xhatminus_data);		
	mat_init(&F->z,4,1,(float *)I->z_data);									
	
	mat_init(&F->A,4,4,(float *)I->A_data);									
	mat_init(&F->AT,4,4,(float *)I->AT_data);								
	mat_trans(&F->A, &F->AT); 															
	
	mat_init(&F->Q,4,4,(float *)I->Q_data);									

	mat_init(&F->H,4,4,(float *)I->H_data);									
  mat_init(&F->HT,4,4,(float *)I->HT_data);								
  mat_trans(&F->H, &F->HT); //转秩

	mat_init(&F->R,4,4,(float *)I->R_data);									
	mat_init(&F->P,4,4,(float *)I->P_data);									
	mat_init(&F->Pminus,4,4,(float *)I->Pminus_data);				
	mat_init(&F->K,4,4,(float *)I->K_data);									
	
	

}

//卡尔曼结构体指针，4维，位置信息x,y，速度信息vx,vy
//F405平台，168Mhz主频下测试，耗时<80us
//注意：各输入参数需要保持单位一致，建议统一为国际单位制
float *kalman_filter_calc(kalman_filter_t *F, float x_in, float y_in, float vx_in, float vy_in)
{
  float TEMP_data[16] = {0};
  float TEMP_data41[4] = {0};
  mat TEMP,TEMP41;
	static u8 state_kalman_init = 0;

	
  mat_init(&TEMP,4,4,(float *)TEMP_data);
  mat_init(&TEMP41,4,1,(float *)TEMP_data41);
	
	if(state_kalman_init == 0)
	{
		state_kalman_init = 1;
		kalman_filter_init(&kalman_filter_F, &kalman_filter_I);
	}
		
	
  F->z.pData[0] = x_in;
  F->z.pData[1] = y_in;
  F->z.pData[2] = vx_in;
  F->z.pData[3] = vy_in;
	
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP41);
  mat_sub(&F->z, &TEMP41, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP41);
  mat_add(&F->xhatminus, &TEMP41, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
//  mat_mult(&F->K, &F->H, &F->P);
//  mat_sub(&F->Q, &F->P, &TEMP);
//  mat_mult(&TEMP, &F->Pminus, &F->P);
  mat_mult(&F->K, &F->H, &F->P);
	mat_mult(&F->P, &F->Pminus, &TEMP);
  mat_sub(&F->Pminus, &TEMP, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];	//滤波之后的位置x
  F->filtered_value[1] = F->xhat.pData[1];	//滤波之后的位置y
	F->filtered_value[2] = F->xhat.pData[2];	//滤波之后的速度vx
  F->filtered_value[3] = F->xhat.pData[3];	//滤波之后的速度vy

  return F->filtered_value;
}

void kalman_filter_reset(kalman_filter_t *F, kalman_filter_init_t *I)
{
	kalman_filter_init(F, I);
	kalman_filter_F.filtered_value[0] = 0;
	kalman_filter_F.filtered_value[1] = 0;
	kalman_filter_F.filtered_value[2] = 0;
	kalman_filter_F.filtered_value[3] = 0;
}
//卡尔曼结构体指针，1维拉尔曼 位置信息   仅为测试
float kalman_filter_calc_1(kalman_filter_1t *F, float signal1)
{

  F->z = signal1;

  //1. xhat'(k)= A xhat(k-1)
	F->xhatminus = F->A * F->xhat;


  //2. P'(k) = A P(k-1) AT + Q
	F->Pminus = F->A * F->P * F->AT + F->Q;


  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
	F->K = F->Pminus * F->HT / (F->H * F->Pminus * F->HT + F->R);


  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	
	F->xhat = F->xhatminus + F->K *(F->z - F->H * F->xhatminus);


  //5. P(k) = (1-K(k)H)P'(k)
	F->P = F->Pminus - F->Pminus * F->K * F->H;

  F->filtered_value = F->xhat;


  return F->filtered_value;
}