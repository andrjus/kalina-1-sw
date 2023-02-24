#include "kalina1_central_common.hpp"
#include "core/robosd_app.hpp"
#include "kalina1-drive.proto.hpp"
#include "servo/mexo_backend.hpp"

#define MODULE_NAME  kalina1
#define MODULE_NAME_STR RT("kalina1")

#define MOTOR_CURRENT_MEASSURY_ENABLED 1
#define MOTOR_POSTITION_MEASSURY_ENABLED KALINA1_MOTOR_ENCO_ENABLED
#define MOTOR_DEV_ERROR_VAR_NAME "pmsm.dev.error"
#include "servo/mexo_motor.inc.hpp"

#include <fstream>

namespace MODULE_NAME{	

	namespace frontend {
		class servo : public mexo::frontend::servo {
			using delegat = ::robo::signal::owned::member< servo >;

			::robo::event on_feedback_ready_;
			delegat on_exchange_complete_;
			std::ifstream  reqFile;
			std::ofstream  actualFile;
			volatile float next_time_ms=5000;
			volatile float next_speed = 0;
			void on_exchange_complete__(void) {
				on_feedback_ready_.raise();
				content.base.action.controller.mode = kalina1_drive::front::modes::off;
				content.base.action.pmsm.actuator.ps.dev.agent.command = ::robo::common::devagent::sw2dirrect;
				content.base.action.pmsm.actuator.ps.current = 1001;
				content.base.action.pmsm.actuator.speed = 500;
				content.base.action.pmsm.actuator.position = 0;

				content.opu.action.controller.mode = kalina1_drive::front::modes::position;
				content.opu.action.pmsm.actuator.ps.current = 1001;
				content.opu.action.pmsm.actuator.speed = 500;
				content.opu.action.pmsm.actuator.position = 0;
				content.opu.action.pmsm.actuator.ps.dev.agent.command = ::robo::common::devagent::sw2dirrect;

				content.shoulder.action.pmsm.actuator.ps.current = 2001;
				content.shoulder.action.pmsm.actuator.ps.dev.agent.command = ::robo::common::devagent::sw2dirrect;

				float tm = (float)robo::system::time_ms();
				if (tm - next_time_ms >= 0) {
					content.shoulder.action.controller.mode = kalina1_drive::front::modes::motion;
					content.shoulder.action.pmsm.actuator.speed = next_speed* ::robo::rad2grad<float>;
					loadReqSpeed_();
				}

				actualFile << std::to_string(tm / 1000. - 10.) << "\t";
				for (int i = 0; i < 6; ++i) {
					actualFile << "\t" << std::to_string(content.drivers[i].feedback.pmsm.actuator.ps.current);
				}
				for (int i = 0; i < 6; ++i) {
					actualFile << "\t" << std::to_string(content.drivers[i].feedback.pmsm.actuator.speed);
				}
				actualFile << std::endl;

				content.elbow.action.controller.mode = kalina1_drive::front::modes::position;
				content.elbow.action.pmsm.actuator.ps.current = 2000;
				content.elbow.action.pmsm.actuator.speed = 500;// next_speed* ::robo::rad2grad<float> * 39;
				content.elbow.action.pmsm.actuator.position = 0;
				content.elbow.action.pmsm.actuator.ps.dev.agent.command = ::robo::common::devagent::sw2dirrect;


				content.wirst_pitch.action.controller.mode = kalina1_drive::front::modes::position;
				content.wirst_pitch.action.pmsm.actuator.ps.current = 1001;
				content.wirst_pitch.action.pmsm.actuator.speed = 500;
				content.wirst_pitch.action.pmsm.actuator.position = 0;
				content.wirst_pitch.action.pmsm.actuator.ps.dev.agent.command = ::robo::common::devagent::sw2dirrect;

			}
			
			void exchange_(void) {
				::robo::frontend::shared::exchange(content, &on_exchange_complete_);
			}

			::robo::frontend::timer timer_;
			robo::time_us_t send_period_us_ = 0;
			void loadReqSpeed_(void) {
				std::string s;
				reqFile >> s;
				if (s.length() > 0) {
					setlocale(LC_NUMERIC, "C");
					next_time_ms = atof(s.c_str()) * 1000 + 10000.;
					reqFile >> s;
					next_speed = atof(s.c_str());
					setlocale(LC_NUMERIC, "");
				}
			}
		protected:

			virtual bool do_load(void) {
				ROBO_LBREAKN(mexo::frontend::servo::do_load());
				ROBO_LBREAKN(::robo::ini::load(current_path(), defaults_path(), RT("send_period_us"), send_period_us_));
				reqFile.open("I:\\SOURCETREE\\CNIIRTK\\KALI\\kali\\content\\doc\\matlab\\kalina1\\exp1w191eps600_req.txt");				
				actualFile.open("I:\\SOURCETREE\\CNIIRTK\\KALI\\kali\\content\\doc\\matlab\\kalina1\\exp1w191eps600_actual.txt");
				content.shoulder.action.controller.mode = kalina1_drive::front::modes::position;
				content.shoulder.action.pmsm.actuator.speed = 500;
				content.shoulder.action.pmsm.actuator.position = -90;
				return true;
			}
			virtual void do_stop(void) {
				actualFile.flush();
			}

		public:
			void config_complete(void) {
				timer_.start(send_period_us_);
			}

			common::content_s content = {};
			servo(robo::app::module & _module)
				:mexo::frontend::servo(RT("fre"), _module) 
				, on_exchange_complete_(*this, &servo::on_exchange_complete__)
				, timer_(
					::robo::signal::autonum::create(*this, &servo::exchange_)
				) {

			}

		};
	}
	
	namespace backend {
		class servo : public mexo::backend::servo {
			robo::quest* config_finish_quest_ = nullptr;
		public:
			typedef ::mexo::backend::motor::agent_t<servo, common::drive_s, kalina1_drive::front::snapshot0_s, kalina1_drive::front::goal0_s> motor;

			motor base;
			motor opu;
			motor shoulder;
			motor elbow;
			motor wirst_pitch;
			motor wirst_yaw;
			motor tool;

			common::content_s& content_;
			virtual ::robo::quest* config_finish_quest() { return config_finish_quest_; };

			servo(robo::app::module& _module, common::content_s& _content, ::robo::quest * _config_finish_quest)
				: mexo::backend::servo(RT("bke"), _module), content_(_content)
				, config_finish_quest_(_config_finish_quest)
				, base(RT("base"), *this, _content.base)
				, opu(RT("opu"), *this, _content.opu)
				, shoulder(RT("shoulder"), *this, _content.shoulder)
				, elbow(RT("elbow"), *this, _content.elbow)
				, wirst_pitch(RT("wirst_pitch"), *this, _content.wirst_pitch)
				, wirst_yaw(RT("wirst_yaw"), *this, _content.wirst_yaw)
				, tool(RT("tool"), *this, _content.tool)
			{
			}
		};

	}
	

	class module : public robo::app::module {
		frontend::servo frontend_;
		backend::servo backend_;
		module(void)
			: robo::app::module(MODULE_NAME_STR)
			, frontend_(*this)
			, backend_(
				*this
				, frontend_.content
				, ::robo::quest::create(
					nullptr
					, nullptr
					, ::robo::quest::answer_fabric::create(
						[this](robo::quest::result r)->robo::quest::reaction {
							if (r == robo::quest::result::success) {
								frontend_.config_complete();
								robo_infolog("=======================================================\n\t\t%s CONFIG SUCCESSED\n=======================================================", this->display_alias());
								return robo::quest::reaction::normal;
							}
							else {
								robo_errlog("=======================================================\n\t\t%s CONFIG REFUSED\n=======================================================", this->display_alias());
								return robo::quest::reaction::terminate;
							}
						}
					)
				)
			)
		{
		}
	protected:
		virtual void backend_loop(void) {}
		virtual void frontend_loop(void) {
		}
		virtual bool do_load(void) { 
			ROBO_LBREAKN(robo::app::module::do_load());			
			kalina1::common::robot_t<joint::yaw> rb;
			joint::actuator::ref* r2 = rb.actuators.first();
			rb.shoulder.move_dg(0);
			rb.update_forvard();
			robo_infolog(
				"actual coords: x=%5.3f\ty=%5.3f\tz=%5.3f\tw=%5.3f\tn=[%5.3f\t%5.3f\t%5.3f]"
				, rb.tool.position.base.offset.x
				, rb.tool.position.base.offset.y
				, rb.tool.position.base.offset.z
				, rb.tool.position.base.orient.w
				, rb.tool.position.base.orient.x
				, rb.tool.position.base.orient.y
				, rb.tool.position.base.orient.z
			);
			return true; 
		}
		virtual bool do_start(void) {
			robo::backend::timer::core::start(nullptr, 10000);			
			
			
			return true;
		}
		virtual void do_stop(void) {
		}
		virtual void do_clean(void) {
		}
	public:
		static module& instance(void) {
			static module instance_;
			return instance_;
		}
	};
}

#include "core/robosd_system_module_reg.hpp" 
