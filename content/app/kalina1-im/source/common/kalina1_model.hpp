#ifndef __warlock_model_hpp
#define  __warlock_model_hpp
#include "core/robosd_system.hpp"
#include "net/robosd_can.hpp"

namespace kalina1_model {
	void periphery_begin(void);
	void periphery_start(void);
	void periphery_stop(void);
	void periphery_finish(void);
}
#endif