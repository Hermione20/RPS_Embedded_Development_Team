//#include "main.h"
#include "stm32f4xx_usart.h"
#include "main.h"
//***************************************函数***************************************************
/*
************************************************************************************************
*函数：unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
*参数：Buf:校验的数据CRC_CNT:校验的位数
*返回：校验结果
*功能：配合串口示波器校验数据
************************************************************************************************
*/



unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
  unsigned short CRC_Temp;
  unsigned char i,j;
  CRC_Temp = 0xffff;
  for (i=0;i<CRC_CNT; i++)
  {      
    CRC_Temp ^= Buf[i];
    for (j=0;j<8;j++) 
    {
      if (CRC_Temp & 0x01)
        CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
      else
        CRC_Temp = CRC_Temp >> 1;
    }
  }
  return(CRC_Temp);
}
/*
************************************************************************************************
*函数：void OutPut_Data(float OutData[4])
*参数：OutData[4]:输出数组
*返回：
*功能： 配合串口示波器的输出协议，float 字长设为16位！！！注意工程
*     1 可以输出4路信息，每路数据长度是16位。
*     2 每次输出字节总数10个，最后两个字节为校验位
************************************************************************************************
*/

  

