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
/** @file ramp.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief ramp contrl realization
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __ramp_second_H__
#define __ramp_second_H__

#include "stm32f4xx.h"

typedef struct ramp_second_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_second_t *ramp_second, int32_t scale);
  float (*calc)(struct ramp_second_t *ramp_second);
}ramp_second_t;

#define RAMP_SECOND_GEN_DAFAULT \
{ \
              .count = 0, \
              .scale = 60, \
              .out = 0, \
              .init = &ramp_second_init, \
              .calc = &ramp_second_calc, \
            } \
            
void  ramp_second_init(ramp_second_t *ramp_second, int32_t scale);
float ramp_second_calc(ramp_second_t *ramp_second);

#endif
