#ifndef kalina1_central_common_hpp
#define kalina1_central_common_hpp

#include "kinematics/robosd_kinematics.hpp"
//#include "servo/mexo_frontend.hpp"

#include "kalina1-drive.proto.hpp"

namespace kalina1 {

	using joint = ::robo::kinematiks::joint<double>;
	using point = ::robo::kinematiks::point<double>;
	using vector3 = ::robo::kinematiks::vector3<double>;
	using quaternion = ::robo::kinematiks::quaternion<double>;
	using avionic = ::robo::kinematiks::axis::avionic<double>;
	using  targpoint = point::hamilton;

	using namespace robo::kinematiks;

	constexpr	double pi = ::robo::pi<double>;
	constexpr double deg2rad = ::robo::kinematiks::deg2rad<double>;
	constexpr double rad2deg = ::robo::kinematiks::rad2deg<double>;
	constexpr double epsilon = ::robo::kinematiks::epsilon<double, 10>;
	constexpr double s2d2 = ::robo::csqrt<double>(2.)/2.;
	namespace common {
		struct drive_s {
			typedef	 kalina1_drive::front::action_t<void> action_s;
			typedef	 kalina1_drive::front::feedback_t<void> feedback_s;
			struct config_s : public ::robo::common::devagent::config_s {
				struct {
					float min;
					float max;
					float offset;
				} pos;
			} config;
			action_s action;
			action_s goal;
			feedback_s feedback;
		};

		struct content_s {
			enum { count = 7 };
			union {
				struct {
					drive_s base;
					drive_s opu;
					drive_s shoulder;
					drive_s elbow;
					drive_s wirst_pitch;
					drive_s wirst_yaw;
					drive_s tool;
				};
				drive_s drivers[count];
			};
		};

		template< typename J > struct robot_t : public joint::series {
		public:
			J	base;
			J	opu;
			J	shoulder;
			J	elbow;
			J	wirst_pitch;
			J	wirst_yaw;
			J	tool;
			/*
			double d1 = 0.1625;
			double a2 = 0.425;
			double a3 = 0.3922;
			double d4 = 0.1333;
			double d5 = 0.0997;
			double d6 = 0.0996;
			*/
			double d1 = 0.089159+0.195;
			double a2 = 0.425;
			double a3 = 0.3923;
			double d4 = 0.10915;
			double d5 = 0.09465;
			double d6 = 0.0823;
			robot_t(void)
				: base(0, *this)
				, opu(1, *this)
				, shoulder(2, *this)
				, elbow(3, *this)
				, wirst_pitch(4, *this)
				, wirst_yaw(5, *this)
				, tool(6, *this) {
				//KALINA1e
				//	Kinematics	theta[rad]	a[m]		d[m]	alpha[rad]	Dynamics	Mass[kg]	Center of Mass[m]
				//	Joint 1		0			0			0.1625	π / 2	Link 1		3.761[0, -0.02561, 0.00193]
				//	Joint 2		0			-0.425		0		0		Link 2		8.058[0.2125, 0, 0.11336]
				//	Joint 3		0			- 0.3922	0		0		Link 3		2.846[0.15, 0.0, 0.0265]
				//	Joint 4		0			0			0.1333	π / 2	Link 4		1.37[0, -0.0018, 0.01634]
				//	Joint 5		0			0			0.0997 - π / 2	Link 5		1.3[0, 0.0018, 0.01634]
				//	Joint 6		0			0			0.0996	0		Link 6		0.365[0, 0, -0.001159]

				base.position.local.orient = quaternion(s2d2, s2d2, 0, 0);
				base.mass = 20.;
				base.ct.local = { 0, 0., 0. };

				opu.position.local.orient = quaternion(s2d2, -s2d2, 0, 0);
				opu.position.local.offset.y = d1;
				opu.mass = 3.761;
				opu.ct.local = { 0, -0.02561, 0.00193 };

				shoulder.mass = 8.058;
				shoulder.ct.local = { -0.2125, 0, 0.11336 };

				shoulder.position.local.offset.y = -0.11;
				shoulder.position.local.orient = quaternion(s2d2, s2d2, 0, 0);
				shoulder.mass = 8.058;
				shoulder.ct.local= { -0.2125, 0, 0.11336 };

				elbow.position.local.offset.x = -a2;
				elbow.mass = 2.846;
				elbow.ct.local = { -0.15, 0.0, 0.0265 };

				wirst_pitch.position.local.offset.x = -a3;
				wirst_pitch.position.local.offset.z = -0.11;
				wirst_pitch.mass = 1.37;
				wirst_pitch.ct.local = { 0, -0.0018, 0.01634 };

				wirst_yaw.position.local.offset.z = d4;
				wirst_yaw.position.local.orient = quaternion(s2d2, s2d2, 0, 0);
				wirst_yaw.mass = 1.3;
				wirst_yaw.ct.local = { 0, 0.0018, 0.01634 };

				tool.position.local.offset.z = d5;
				tool.position.local.offset.y = d6;
				tool.position.local.orient = quaternion(s2d2, -s2d2, 0, 0);
				tool.mass = 0.365;
				tool.ct.local = { 0, 0, -0.001159 };
			}
		};
	}
}
#endif