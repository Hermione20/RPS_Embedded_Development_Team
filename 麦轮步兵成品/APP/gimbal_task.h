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
/** @file gimbal_task.h
 *  @version 1.1
 *  @date Oct 2017
 *
 *  @brief gimbal control task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__
#include "main.h"
/* gimbal control period time (ms) */
#define GIMBAL_PERIOD 5
#define FILTER_NUM 5
#if STANDARD == 3
#define small_buff_y_offset   155     //增大偏上
#define small_buff_x_offset   -20

#elif STANDARD == 4
#define small_buff_y_offset   190
#define small_buff_x_offset   -15

#elif STANDARD == 5
//#define small_buff_y_offset   60
//#define small_buff_x_offset   -15
#define small_buff_y_offset   105
#define small_buff_x_offset   -35

#elif STANDARD == 6
//#define small_buff_y_offset   60
//#define small_buff_x_offset   -15
#define small_buff_y_offset   105
#define small_buff_x_offset   -35
#endif

typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
  GIMBAL_AUTO_AIM	   = 8,
  GIMBAL_AUTO_SUP      = 9,
  GIMBAL_REVERSE       = 10,
  GIMBAL_AUTO_SMALL_BUFF   = 11,
  GIMBAL_AUTO_BIG_BUFF     = 12,
  GIMBAL_AUTO_ANGLE    = 13,
  GIMBAL_FOLLOW_CHASSIS=14,
} gimbal_mode_e;

typedef enum
{
  NO_ACTION = 0,
  IS_ACTION,
} action_mode_e;

typedef enum
{ 
  CMD_NO =0,
  CMD_CALI_FIVE,
  CMD_CALI_NINE,
  CMD_TARGET_NUM
} gimbal_cmd_e;



typedef struct
{
  /* position loop */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* speed loop */
  float yaw_speed_ref;
  float pit_speed_ref;
  float yaw_speed_fdb;
  float pit_speed_fdb;
} gim_pid_t;

typedef struct
{
  /* unit: degree */
  float pit_relative_angle;
  float yaw_relative_angle;
  float gyro_angle;
  /* uint: degree/s */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

typedef struct
{
  action_mode_e ac_mode;
  float         action_angle;
  uint8_t       no_action_flag;
  uint32_t      no_action_time;
} no_action_t;

typedef struct
{
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  /* gimbal information */
  gim_sensor_t  sensor;
  float         ecd_offset_angle;
  float         yaw_offset_angle;
  
  /* gimbal ctrl parameter */
  gim_pid_t     pid;
  no_action_t   input;
  
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  
  gimbal_cmd_e  auto_ctrl_cmd;
} gimbal_t;



typedef enum
{
    NORMAL_SHOOT    = 0,
    AUTO_SHOOT      = 1,
} auto_shoot_mode_e;


typedef struct
{
    float Yaw_Angle_Pre;
    float Yaw_Angle_Now;
    float Pit_Angle_Pre;
    float Pit_Angle_Now;
    uint32_t Time_Sample;
	  float Angular_Yaw_Speed;
	  float Angular_Yaw_Speed_Pre;
    float Angular_Pit_Speed;
	  float Angular_Pit_Speed_Pre;
	  float Yaw_Acceleration;
	  float Pit_Acceleration;
	  uint32_t time1;
	  uint32_t time2;
	  float time_error;
	  float yaw_angle_error;
	  float pit_angle_error;
	  float time_delay;
} Speed_Prediction_t;

typedef struct//自瞄所用参数
{
    Speed_Prediction_t Speed_Prediction;
	  Speed_Prediction_t Speed_Prediction_Kalman;
    float Filtered_Angular_Yaw_Speed;
    float Filtered_Angular_Pit_Speed;
	  float Filtered_Yaw_Acceleration;
    float Filtered_Pit_Acceleration;
    uint8_t	  Recognized_Flag;
    uint16_t   Recognized_Timer;
	  int16_t   Continue_Recognized_Cnt;
    float Err_Pixels_Yaw;
    float Err_Pixels_Pit;
  	float Distance;
    float Ballistic_Compensation;
    float Yaw_Gimbal_Delay_Compensation;
	  float Pit_Gimbal_Delay_Compensation;
	  uint8_t Image_Gimbal_Delay_Compensation_Flag;
    float Delta_Dect_Angle_Pit;
    float Delta_Dect_Angle_Yaw;
	  uint16_t   Continue_Large_Err_Cnt;
	  double shoot_angle;
	  float Amror_yaw;
	  float Amror_pit;
} Gimbal_Auto_Shoot_t;

extern float click_x,click_y;
extern float yaw_angle_ref;
extern float now_distance ;
extern float last_distance ;
extern auto_shoot_mode_e  autoshoot_mode;
extern gimbal_t gim;
extern receive_judge_t judge_rece_mesg;
extern Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;

extern float YAW_ANGLE_BETWEEN_GUN_CAMERA ;
extern float PITCH_ANGLE_BETWEEN_GUN_CAMERA;

extern float shoot_angle;
extern float pitch_remain;
static void init_mode_handle(void);
static void close_loop_handle(void);
static void gimbal_patrol_handle(void);
static void big_buff_handle(void);

static void cascade_pid_ctrl(void);
void gimbal_param_init(void);
void gimbal_back_param(void);
void gimbal_task(void);
void auto_aim_cascade_pid_ctrl(void);
void auto_small_buff_handle(void);
void auto_big_buff_handle(void);
void auto_angle_handle(void);//吊射
void gimbal_follow_chassis_handle(void);

float AvgFilter(float new_value);
float raw_data_to_pitch_angle(float ecd_angle_pit);
extern u8 key_f_pitch;
extern u8 key_e_yaw;
#endif