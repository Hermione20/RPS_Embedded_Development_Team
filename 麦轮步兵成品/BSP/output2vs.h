#ifndef _OUTPUT2VS_H_
#define _OUTPUT2VS_H_

#include "main.h"

//声明变量
extern int OutData[4];//串口示波器 全局变量输出数据

//声明函数
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);//数据校验    
void OutPut_Data(int OutData[4]);//数据输出到示波器显示

#endif
//****************************************END***************************************************
