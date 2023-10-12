/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file pid.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief pid parameter initialization, position and delta pid calculate
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
	
#include "oldpid.h"
#include "math.h"

void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

  pid->integral_limit = intergral_limit;
  pid->max_out        = maxout;
  pid->pid_mode       = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
  
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output 
  */
float pid_calc(pid_t *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get;

  if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
      return 0;

  if (pid->pid_mode == POSITION_PID) //position PID
  {
      pid->pout = pid->p * pid->err[NOW];
      pid->iout += pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    
      abs_limit(&(pid->iout), pid->integral_limit);
      pid->out = pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }
  else if (pid->pid_mode == DELTA_PID) //delta PID
  {
      pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
      pid->iout = pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

      pid->out += pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  
  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;

}

float pid_calc1(pid_t *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get;

  if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
      return 0;

  if (pid->pid_mode == POSITION_PID) //position PID
  {
      pid->pout = pid->p * pid->err[NOW];
      pid->iout += pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    
      abs_limit(&(pid->iout), pid->integral_limit);
      pid->out = pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }
  else if (pid->pid_mode == DELTA_PID) //delta PID
  {
      pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
      pid->iout = pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

      pid->out += pid->pout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  
  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;

}

/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
  pid->f_param_init = pid_param_init;
//  pid->f_pid_reset  = pid_reset;

  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
//  pid->f_pid_reset(pid, kp, ki, kd);
}



/**
  * @brief     clear pid out
  * @retval    none
  */
void pid_clr(pid_t *pid)
{
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
	pid->err[0] = 0;
  pid->err[1] = 0;
	pid->err[2] = 0;
}


float pid_double_loop_cal(pid_t *Outer_loop_pid,
                          pid_t *Inner_loop_pid,
                          float outer_ref,
                          float outer_fdb,
													float *Inner_ref,
                          float Inner_fdb,
                          float feedforward)
{
	
  *Inner_ref = pid_calc(Outer_loop_pid,outer_fdb,outer_ref) + feedforward;
  return pid_calc(Inner_loop_pid,Inner_fdb,*Inner_ref);
}



//自瞄模式外环的参数
pid_t pid_yaw_follow    = {0};
pid_t pid_pit_follow    = {0};
pid_t pid_pit_speed_follow    = {0};
pid_t pid_yaw_speed_follow    = {0};


//小符下的PID参数
pid_t pid_yaw_small_buff    = {0};
pid_t pid_pit_small_buff    = {0};
pid_t pid_pit_speed_small_buff    = {0};
pid_t pid_yaw_speed_small_buff    = {0};

pid_t pid_yaw_small_buff1    = {0};
pid_t pid_pit_small_buff1    = {0};
pid_t pid_pit_speed_small_buff1    = {0};
pid_t pid_yaw_speed_small_buff1    = {0};

//大符下的PID参数
pid_t pid_yaw_big_buff    = {0};
pid_t pid_pit_big_buff    = {0};
pid_t pid_pit_speed_big_buff    = {0};
pid_t pid_yaw_speed_big_buff    = {0};

pid_t pid_yaw_big_buff1    = {0};
pid_t pid_pit_big_buff1    = {0};
pid_t pid_pit_speed_big_buff1    = {0};
pid_t pid_yaw_speed_big_buff1    = {0};

//吊射下的PID参数
pid_t pid_yaw_auto_angle    = {0};
pid_t pid_pit_auto_angle    = {0};
pid_t pid_pit_speed_auto_angle    = {0};
pid_t pid_yaw_speed_auto_angle    = {0};


//自动补弹的PID参数
pid_t pid_yaw_follow_chassis_angle    = {0};
pid_t pid_yaw_follow_chassis_speed    = {0};


//普通模式PID参数
pid_t pid_yaw           = {0};
pid_t pid_pit           = {0};
pid_t pid_auto_aim_yaw  = {0};
pid_t pid_auto_aim_pit  = {0};
pid_t pid_yaw_speed     = {0};
pid_t pid_pit_speed     = {0};
pid_t pid_spd[4]        = {0};
pid_t pid_chassis_angle = {0};
pid_t pid_trigger       = {0};
pid_t pid_trigger_speed = {0};
pid_t pid_trigger_second_speed = {0};
pid_t pid_rotate[4]     = {0};
pid_t pid_imu_tmp       = {0};
pid_t pid_voltage       = {0};
pid_t pid_software_limit= {0};
pid_t pid_speed_bias    = {0};
pid_t pid_front_distance= {0};
pid_t pid_right_distance= {0};
pid_t pid_angle_distance= {0};
pid_t pid_spring[2] = {0};

pid_t pid_cha_6020_angle[4]={0};
pid_t pid_cha_3508_angle[4]={0};
pid_t pid_cha_6020_speed[4]={0};
pid_t pid_cha_3508_speed[4]={0};