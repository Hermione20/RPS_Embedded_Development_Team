//#include "main.h"

//spring_State_e SPRING_State=SPRING_NORMAL_STATE;
//u32 spring_tick=0;
//void spring_task(void)
//{	
//	switch(SPRING_State)
// {
//		case SPRING_NORMAL_STATE:
//			pid_spring[0].set=0;
//			pid_spring[1].set=0;
//	    spring_flag=0;
//     if(spring_flag==1)
//	 {
//	   SPRING_State=SPRING_STATE_1;
//	 }	
//     if(spring_flag==2)
//	 {
//	  SPRING_State=SPRING_STATE_2;
//	 }	
//		break;
//	 
//	 case SPRING_STATE_1://╫Т
//			pid_spring[0].set=3000;
//			pid_spring[1].set=-3000;
//	        spring_tick++;
//	    if(spring_tick>500)
//		  {
//			  pid_spring[0].set=0;
//			  pid_spring[1].set=0;
//			  spring_tick=0;
//				SPRING_State=SPRING_NORMAL_STATE;
//		  }

//		break;
//	 
//       case SPRING_STATE_2://ки
//			    pid_spring[0].set=-3000;
//			    pid_spring[1].set=3000;
//	        spring_tick++;
//	      if(spring_tick>500)
//		  {
//			  pid_spring[0].set=0;
//			  pid_spring[1].set=0;
//			  spring_tick=0;
//				SPRING_State=SPRING_NORMAL_STATE;
//		  }

//		break;
// }
//  spring_handle();
//}


//void spring_handle(void)

//{

//    pid_spring[0].get=BUS1_spring_CM5Encoder.filter_rate;//
//    pid_spring[1].get=BUS1_spring_CM6Encoder.filter_rate;//
//	  pid_calc(&pid_spring[0],pid_spring[0].get, pid_spring[0].set);
//	  pid_calc(&pid_spring[1],pid_spring[1].get, pid_spring[1].set);
//     SPRING_State=SPRING_NORMAL_STATE;
//	Set_CM_Speed(CAN1, pid_rotate[2].out , pid_rotate[1].out,pid_spring[0].out,pid_spring[1].out);


//}

//void spring_param_init(void)
//{
//	PID_struct_init(&pid_spring[0], POSITION_PID,5000, 5000,10.0f,0.0f,1.0f);
//	PID_struct_init(&pid_spring[1], POSITION_PID,5000, 5000,10.0f,0.0f,1.0f);
//}
//	
