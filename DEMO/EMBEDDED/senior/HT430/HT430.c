#include "HT430.h"



HT430_J10_t HT430_J10;



/********************************HT430_J10************************************/

void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v)
{
  switch ((msg->StdId&0xfffe)>>4)
	{
		case 0x2f:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x40:
		{
			HT430_J10_t->Voltage=msg->Data[0]*0.2;
			HT430_J10_t->Currents=msg->Data[1]*0.03;
			HT430_J10_t->Temperature=msg->Data[2]*0.4;
			HT430_J10_t->DTC=msg->Data[3];
			HT430_J10_t->Operating_State=msg->Data[4];
		}break;
		
		case 0x53:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x54:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=(msg->Data[7]<<8|msg->Data[6]);
		}break;
		
		case 0x55:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x56:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x57:
		{
//			HT430_J10_t->V=(msg->Data[1]<<8|msg->Data[0])*0.1;
		}break;
		
		default:
		{
		}break;
	}
	HT430_J10_t->V=HT430_J10_t->V*360/16384/6;
	v->ecd_angle = HT430_J10_t->Total_Angle;
	v->filter_rate = HT430_J10_t->V;
}
