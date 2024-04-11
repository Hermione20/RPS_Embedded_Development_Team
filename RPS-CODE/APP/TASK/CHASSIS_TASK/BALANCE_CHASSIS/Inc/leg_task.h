#ifndef __LEG_TASK_H
#define __LEG_TASK_H
#include "public.h"



#ifndef LEG_STATE
#define LEG_STATE

typedef struct
{
	float pos[2];//pos=[l0; phi0];
	float spd[2];//spd[2]=[dl0; dphi0];
	float T[2];//T[2]=[motor4;motor1];

	//支持力解算用计算变量
	float J[4];
	float j[2][2];
	float F_fdb;
	float Tp_fdb;

	float phi4;
	float phi1;
	float dphi4;
	float dphi1;

	float this_dl0;
	float last_dl0;

	float l0;
	float dl0;
	float ddl0;
	float phi0;
	float dphi0;

	float leg_F;
	float ddzw;
	float leg_FN;
	float leg_final_FN;
	
	uint8_t wheel_state;

	pid_t leglengthpid;
}leg_state_t;

#endif // !LEG_STATE

void FN_calculate(leg_state_t *leg,float MT1_torque,float MT4_torque);
void VMC_data_get(leg_state_t *leg , float phi4_angle, 
                                    float phi4_gyro,
                                    float phi1_angle,
                                    float phi1_gyro);




void leg_pos(float phi1, float phi4, float pos[2]);
void leg_spd(float dphi1, float dphi4, float phi1, float phi4,
    float spd[2]);
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2]);
void leg_J_cal(float phi1, float phi4, float J[4]);

#endif // !1
