#ifndef kalina1_shared_h
#define kalina1_shared_h
#include "core/robosd_common.hpp"
namespace kalina1_shared{
	constexpr robo::cstr shared_name = RT("kalina1_shared");
	enum class eside  { startup = 0, digitwin = 1, vrep = 2, shutdown = 3 } ;
	struct drill {
		enum { count =2};
		double rotator;
		double supply;
		double power;
		int mode;
	};
	struct content{
		eside side;
		struct {
			double time;
		} vrep;
		union {
			struct {
				drill drill1;
				drill drill2;
			};
			drill drills[drill::count];
		};
	};
	constexpr ::robo::cstr link_names[drill::count] = {
		RT("drill1.load")
		, RT("drill2.load")
	};
	constexpr ::robo::cstr drill_names[drill::count] = {
		RT("drill1")
		, RT("drill2")
	};
}
#endif