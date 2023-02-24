#include "im/edev/edev.hpp"
#include "im/edev/joint_link.hpp"
#include "im/models/gearbox.hpp"
#include "kalina1_model.hpp"
#include "core/robosd_common.hpp"
#include "core/robosd_ini.hpp"
#include "core/robosd_system.hpp"
#include "net/platform/can/emulator/emu_can.hpp"
#include "kalina1_shared.hpp"
#include <chrono>
namespace kalina1_model {

	class agent :public robo::edev::agent {

		::robo::edev::joint::iload yaw;
		robo::time_ms_t send_period_ms_ = 0;
		int num_ = 0;
		double next_time_ = 0.;
		double send_period_ = 0.;
		robo::system::shared shared_;
		kalina1_shared::content  * content_;
		

		::robo::edev::joint::iload motor1;
		::robo::edev::joint::iload motor2;
		::robo::edev::joint::iload* loads[kalina1_shared::drill::count] = {
			&motor1
			, & motor2
		};

	protected:

		virtual void do_background_run(double _time) {
			if (_time >= next_time_) {
				next_time_ += send_period_;
			}
		}

		virtual void do_priotitet_run(double _time) {
			static double last=0;
			if (_time - last > sample_time) {
				auto begin = std::chrono::steady_clock::now();
				last += sample_time;
				::robo::edev::joint::iload** ld = loads;
				for (int i = 0; i < kalina1_shared::drill::count; ++i, ++ld) {
					::robo::edev::joint::iload* l = *ld;
					l->speed =
						l->driveng_speed;
					l->position =
						l->driveng_position;
				}

				switch (content_->side) {
				case kalina1_shared::eside::shutdown:
				break;
				case kalina1_shared::eside::vrep:
				break;
				case kalina1_shared::eside::startup:
				case kalina1_shared::eside::digitwin:
				content_->vrep.time = _time;
				{
					::robo::edev::joint::iload** ld = loads;
					kalina1_shared::drill* a = content_->drills;
					for (int i = 0; i < kalina1_shared::drill::count; ++i, ++ld, ++a) {
						a->rotator = (*ld)->position;
						a->supply = 0.;
						a->power = 0.;
					}
				}
				content_->side = kalina1_shared::eside::vrep;
				break;
				}				
			}
		}

		virtual bool do_begin(void) {
			periphery_begin();
			ROBO_LBREAKN(robo::edev::agent::do_begin());
			//ROBO_LBREAKN(::robo::ini::load(name, RT("SEND_PERIOD_MS"), send_period_ms_));
			send_period_ = 0.001 * send_period_ms_;
			periphery_start();
			ROBO_LBREAKN(shared_.open(kalina1_shared::shared_name,sizeof(kalina1_shared::content)) );
			content_ = (kalina1_shared::content*) shared_.memo();
			*content_ = {};
			content_->side = kalina1_shared::eside::startup;
			return  true;
		}


		virtual void do_reconfig(void) {
			for (int i = 0; i < kalina1_shared::drill::count; ++i) {
				robo::edev::joint::gearbox::elastic::friction* ld =
					dynamic_cast<robo::edev::joint::gearbox::elastic::friction*>(::robo::edev::joint::link::find(kalina1_shared::link_names[i]));
				ROBO_VBREAKN(ld != nullptr);
				ld->connect_to_load(loads[i]);
				loads[i]->position = ld->actuator->position / ld->driver.gear_ratio;
			}
		};

		virtual void do_finish(void) {
			periphery_stop();
			periphery_finish();
			shared_.close();
		}
	public:
		agent(void){}
		~agent() {}
		virtual void set_local_ini(robo::cstr _ini) { ::robo::system::ini::begin(_ini); }
	} agent_;
}	


extern "C" {
	ROBO_EXPORT_RUNTIME robo::edev::agent* ROBO_EXPORT_RUNTIME_DECL query_agent(void) {
		return & kalina1_model::agent_;
	}
}

