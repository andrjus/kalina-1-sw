#ifndef k1_burst_servo_proto_h
#define k1_burst_servo_proto_h
#include "stdint.h"

#pragma pack(push,1)
typedef union servo_action_u{
	struct{
		uint8_t mode;
		uint8_t rsrv;
		int16_t voltage;
		int16_t current;
		int16_t speed;
	};
	uint32_t value[2];
} servo_action_t;

typedef union servo_feedback_u{
	struct{
		uint8_t mode;
		uint8_t rsrv;
		int16_t voltage;
		int16_t current;
		int16_t speed;
	};
	uint8_t memo[8];
} servo_feedback_t;

typedef struct servo_actions_s{
	uint16_t values[4];
} servo_actions_t;

enum{
	k1_proto_cmd_serial = 0xF
};
		
#pragma pack ( pop)


#endif
