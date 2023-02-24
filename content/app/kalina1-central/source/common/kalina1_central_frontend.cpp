#include "kalina1_central_frontend.hpp" 
/*
namespace kalina1 {
	robot::robot(void)
		: yaw_1(1, *this)
		, pitch_2(2, *this)
		, pitch_3(3, *this)
		, pitch_4(4, *this)
		, roll_5(5, *this)
		, pitch_6(6, *this)
		, path_(*this) {
		pitch_2.local.orient = axis::avionic<double>(0, pi / 2, 0);
		pitch_2.local.position.z = d1;
		pitch_3.local.position.x = a2;
		pitch_4.local.position.x = a3;
		roll_5.local.position.y = -d4;
		pitch_6.local.position.x = a5;
		pitch_6.local.position.y = -d6;
	}

	void robot::solution::calc_score(const robot& _fit_rb, const targpoint& _target) {
		static robot  _test_rb;
		status* S = statuses;
		score.move = 0.;
		success = true;
		int n = 0;
		joint::actuator::ref* r = _fit_rb.actuators.first();
		joint::actuator::ref* r2 = _test_rb.actuators.first();

		for (int n = 0; n < actuator_count; r = r->next(), ++S, ++n, r2 = r2->next()) {
			bool tmp = S->succes = r->owner().check(S->position, S->delta);
			S->score.move = S->delta * S->delta;
			if (!tmp) {
				success = false;
				score.move = 0.0;
			}
			if (success) {
				score.move += S->score.move;
			}
			r2->owner().move(S->position * rad2deg);
		}

		score.move = sqrt(score.move / (int)actuator_count);

		if (success) {
			_test_rb.assign(_fit_rb);
			_test_rb.update_forvard();
			quaternion dq = _target.orient * _test_rb.pitch_6.base.orient.inv();
			error.orient = _test_rb.pitch_6.base.orient.compare(_target.orient) * _fit_rb.path_.ro;
			error.position = _test_rb.pitch_6.base.position.diff(_target.position);
		}
	}
	robot::path::path(const robot& _rb)
		: rb(_rb)
		, dir_up_(*this)
		, dir_down_(*this)
		, inv_up_(*this)
		, inv_down_(*this) {}

	void robot::path::run(const targpoint& _target) {

		//double x = _target.position.x;
		//double y = _target.position.y;
		//double z = _target.position.z;
		//double ro2 = x * x + y * y + z * z;

		Pg = vector3({ -rb.grab.local.position.x, 0, 0 });
		//Pz = vector3({ 0, 0, -1 });

		//Pg = ::robo::kinematiks::quaternion<double>(-rb.grab.local.position.x, 0, 0);
		//orient = target.orient;
		target = _target;
		V4 = Pg.back_transform(target.orient, quaternion(target.orient, true), target.position);
		//Vz = Pz.back_transform(target.orient, quaternion(target.orient, true), target.position);

		ro2 = V4.dot(V4);

		ro = sqrt(ro2);
		const double epsilon = 0.0000001;
		if (ro < rb.L1 + rb.L2 + epsilon) {

			h = sqrt(V4.x * V4.x + V4.y * V4.y);

			//ro2 = L1*L1+L2*L2-2L1*L2*cos(beta)
			if (V4.y * V4.y + V4.x * V4.x > 0.1) {
				phy = atan2(V4.y, V4.x);
				dummy_phy = false;
			}
			else {
				phy = rb.yaw_1.get();
				dummy_phy = true;
			}
			if (abs(ro - rb.L1 - rb.L2) > epsilon) {
				beta = acos(-(ro2 - rb.L1 * rb.L1 - rb.L2 * rb.L2) / (2 * rb.L1 * rb.L2));
				//sin(alfa) / L2 = sin(beta)/ro
				alfa = asin(sin(beta) / ro * rb.L2);
			}
			else {
				beta = pi;
				alfa = 0.;
			}
			gama = atan2(V4.z, h);

			//вектор от  кисти к схвату
			//L3 = vector3(target.position) - Q4;
			//dirL3 = L3.direction();
			dir_up_.run();
			dir_down_.run();
			inv_up_.run();
			inv_down_.run();

			solutions_[0].yaw_1.position = solutions_[1].yaw_1.position = dir_up_.yaw_1;
			solutions_[0].pitch_2.position = solutions_[1].pitch_2.position = dir_up_.pitch_2;
			solutions_[0].pitch_3.position = solutions_[1].pitch_3.position = dir_up_.pitch_3;
			solutions_[0].roll_4.position = dir_up_.dirrect.roll_4;
			solutions_[0].pitch_5.position = dir_up_.dirrect.pitch_5;
			solutions_[1].roll_4.position = dir_up_.inverce.roll_4;
			solutions_[1].pitch_5.position = dir_up_.inverce.pitch_5;

			solutions_[2].yaw_1.position = solutions_[3].yaw_1.position = dir_down_.yaw_1;
			solutions_[2].pitch_2.position = solutions_[3].pitch_2.position = dir_down_.pitch_2;
			solutions_[2].pitch_3.position = solutions_[3].pitch_3.position = dir_down_.pitch_3;
			solutions_[2].roll_4.position = dir_down_.dirrect.roll_4;
			solutions_[2].pitch_5.position = dir_down_.dirrect.pitch_5;
			solutions_[3].roll_4.position = dir_down_.inverce.roll_4;
			solutions_[3].pitch_5.position = dir_down_.inverce.pitch_5;

			solutions_[4].yaw_1.position = solutions_[5].yaw_1.position = inv_up_.yaw_1;
			solutions_[4].pitch_2.position = solutions_[5].pitch_2.position = inv_up_.pitch_2;
			solutions_[4].pitch_3.position = solutions_[5].pitch_3.position = inv_up_.pitch_3;
			solutions_[4].roll_4.position = inv_up_.dirrect.roll_4;
			solutions_[4].pitch_5.position = inv_up_.dirrect.pitch_5;
			solutions_[5].roll_4.position = inv_up_.inverce.roll_4;
			solutions_[5].pitch_5.position = inv_up_.inverce.pitch_5;

			solutions_[6].yaw_1.position = solutions_[7].yaw_1.position = inv_down_.yaw_1;
			solutions_[6].pitch_2.position = solutions_[7].pitch_2.position = inv_down_.pitch_2;
			solutions_[6].pitch_3.position = solutions_[7].pitch_3.position = inv_down_.pitch_3;
			solutions_[6].roll_4.position = inv_down_.dirrect.roll_4;
			solutions_[6].pitch_5.position = inv_down_.dirrect.pitch_5;
			solutions_[7].roll_4.position = inv_down_.inverce.roll_4;
			solutions_[7].pitch_5.position = inv_down_.inverce.pitch_5;
			select_best(target);
		}
		bool robot::path::select_best(const targpoint & target) {}

	}
}
*/