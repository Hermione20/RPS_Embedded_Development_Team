#include "PM01.h"



void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (msg->Data[0] << 8) | msg->Data[1];
		v->mode_sure = (msg->Data[2] << 8) | msg->Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (msg->Data[0] << 8) | msg->Data[1];
		v->in_v = (msg->Data[2] << 8) | msg->Data[3];
		v->in_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (msg->Data[0] << 8) | msg->Data[1];
		v->out_v = (msg->Data[2] << 8) | msg->Data[3];
		v->out_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x613:
	{
		v->tempureture=(msg->Data[0]<<8)|msg->Data[1];
		v->time=(msg->Data[2]<<8)|msg->Data[3];
        v->this_time=(msg->Data[4]<<8)|msg->Data[5];
	}break;

	default:
		break;
	}
}


