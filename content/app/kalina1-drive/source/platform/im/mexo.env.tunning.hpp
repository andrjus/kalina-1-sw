#if (!defined(mexo_env_tunning_hpp)) && defined(mexo_env_common_hpp)
#define mexo_env_tunning_hpp
#else
#error error of using mexo.env.tunning.hpp
#endif

#include "kalina1-drive.common.hpp"

#define ENV_SAMPLE_US KALINA1_PWM_PERIOD_US

#define ENV_SWITCH_PORT0_SERIAL_PATH RT("CAN_SERIAL0")
#define ENV_SWITCH_PORT0_TYPE ::robo::net::proto::switcher::port::PACKET
#define ENV_SWITCH_PORT1_SERIAL_PATH RT("UART0")
#define ENV_SWITCH_PORT1_TYPE ::robo::net::proto::switcher::port::SERIAL
#define ENV_FREEMASTER_CONNECT_TYPE ENV_FREEMASTER_CONNECT_TYPE_ABONENT
#define ENV_TERMO_CONNECT_TYPE ENV_TERMO_CONNECT_TYPE_ABONENT

#define ENV_NET_FLOW_TYPE ENV_NET_FLOW_TYPE_DEFAULT
#define ENV_NET_FLOW_PORT_PATH  RT("CAN0")

#define ENV_NET_FLOW_ECHO_ENABLED		1
#define ENV_NET_FLOW_SERIAL0_ENABLED	1
#define ENV_NET_FLOW_SERIAL0_PATH	ENV_SWITCH_PORT0_SERIAL_PATH
#define ENV_NET_FLOW_VAR_ENABLED		1


#define ENV_NET_FLOW_ECHO_SUBA			KALINA1_NET_FLOW_ECHO_SUBA
#define ENV_NET_FLOW_ECHO_SUBA_ANSW		KALINA1_NET_FLOW_ECHO_SUBA_ANSW
#define ENV_NET_FLOW_VAR_SUBA			KALINA1_NET_FLOW_VAR_SUBA
#define ENV_NET_FLOW_VAR_SUBA_ANSW		KALINA1_NET_FLOW_VAR_SUBA_ANSW
#define ENV_NET_FLOW_SERIAL0_SUBA		KALINA1_NET_FLOW_SERIAL0_SUBA
#define ENV_NET_FLOW_SERIAL0_SUBA_ANSW	KALINA1_NET_FLOW_SERIAL0_SUBA_ANSW