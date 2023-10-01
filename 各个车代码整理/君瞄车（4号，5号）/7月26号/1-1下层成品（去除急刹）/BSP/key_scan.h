#ifndef __KEY_SCAN_H__
#define __KEY_SCAN_H__



#define SCAN_TIME 10 //
#define DOWN_TIME 10//
#define HOLD_TIME (long unsigned int)4000//

typedef enum key_states_e
{      
	KEY_DOWN,
    KEY_UP

} key_states_e;

typedef enum  
{
    KEY_S1,
    KEY_S2,
    KEY_S3,
	KEY_S4
}key_msg_e;


key_states_e key_read(void);
key_msg_e key_scan(void);
void key_init(void);

#endif  

