#ifndef kalina1_central_frontend_hpp
#define kalina1_central_frontend_hpp

#include "kalina1_central_common.hpp"

	/*
		struct solution {
			enum { count = 8, actuator_count = 6 };
			struct status {
				double position;
				bool succes;
				double delta;
				struct {
					double move;
					double support;
				} score;
			};
			union {
				struct {
					status yaw_1;
					status pitch_2;
					status pitch_3;
					status pitch_4;
					status roll_5;
					status pitch_6;
				};
				status statuses[actuator_count];
			};
			struct {
				double move;
				double support;
			} score;
			struct {
				double position = 0;
				double orient = 0;
			} error;
			bool success = false;
			void calc_score(const robot& _fit_rb, const targpoint& _target);
		};
	private:
		struct wrist;
		struct shoulder;
		struct path;
		struct wrist {
			double roll_4 = 0.;
			double pitch_5 = 0.;
			shoulder& sh;
			wrist(shoulder& _sh) :sh(_sh) {}
			virtual void run(void) = 0;
		};
		struct wrs_dir :public wrist {
			wrs_dir(shoulder& _sh)
				: wrist(_sh) {}
			virtual void run(void);
		};
		struct wrs_inv :public wrist {
			wrs_inv(shoulder& _sh)
				: wrist(_sh) {}
			virtual void run(void);
		};
		struct shoulder {
			path& ph;
			double yaw_1 = 0.;
			double pitch_2 = 0.;
			double pitch_3 = 0.;
			quaternion IQ4;
			quaternion Q46;

			double wrist_pitch = 0.;
			double wrist_roll = 0.;
			double wrist_roll_2 = 0.;
			bool dummy_roll = false;

			void apply(void);
			wrs_dir dirrect;
			wrs_inv inverce;
			shoulder(path& _ph)
				: ph(_ph)
				, dirrect(*this)
				, inverce(*this) {}
			virtual void run(void) = 0;
		};
		struct sh_dir_up :public shoulder {
			sh_dir_up(path& _ph)
				: shoulder(_ph) {}
			virtual void run(void);
		};
		struct sh_inv_up :public shoulder {
			sh_inv_up(path& _ph)
				: shoulder(_ph) {}
			virtual void run(void);
		};
		struct sh_dir_down :public shoulder {
			sh_dir_down(path& _ph)
				: shoulder(_ph) {}
			virtual void run(void);
		};
		struct sh_inv_down :public shoulder {
			sh_inv_down(path& _ph)
				: shoulder(_ph) {}
			virtual void run(void);
		};

		struct path {
			const robot& rb;

			double ro2 = 0.;
			double ro = 0.;
			double h = 0.;
			double alfa = 0.;
			double beta = 0.;
			double gama = 0.;
			double phy = 0.;
			bool dummy_phy = false;
			int best_index = -1;

			targpoint target;

			vector3 Pg;
			vector3	V4;


			sh_dir_up dir_up_;
			sh_dir_down dir_down_;
			sh_inv_up inv_up_;
			sh_inv_down inv_down_;

			solution solutions_[solution::count];

			path(const robot& _rb);

			void run(const targpoint& target);
			bool select_best(const targpoint& target);
		} path_;
	};
}
*/
#endif