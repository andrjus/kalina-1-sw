#ifndef kalina1_drive_proto_hpp
#define kalina1_drive_proto_hpp
#include "kalina1-drive.common.hpp"

#define PMSM_TEMPLATE_NAME KALINA1_DRIVE_PMSM_NAME
#define PMSM_ACTUATOR_SUB_NAME KALINA1_DRIVE_ACTUATOR_NAME
#define PMSM_PS_CROSS_NAME KALINA1_DRIVE_PS_CROSS_NAME
#define PMSM_PS_LAT_NAME KALINA1_DRIVE_PS_LAT_NAME
#include "mexo/pmsm.templ.front.inc.hpp"

#if KALINA1_BRAKE_ENABLED == 1
#define BRAKE_TEMPLATE_NAME KALINA1_BRAKE_NAME
#define BRAKE_PS_TEMPLATE_NAME KALINA1_BRAKE_PS_NAME
#include "mexo/brake.templ.front.inc.hpp"
#endif

#include "mexo/math.hpp"

namespace kalina1_drive {
	namespace front {
		enum class  modes { off = 0, position = 4, motion = 3, tractor = 5, passive = 1, unknown = 255 };
		enum class  statuses { none = 0, run = 1, success = 2, terminated = 3, unknown = 255 };
		static inline modes decode_mode(uint8_t _mode) { return _mode <= (uint8_t)modes::tractor ? (modes)_mode : modes::unknown; }
		static inline statuses decode_status(uint8_t _status) { return _status <= (uint8_t)statuses::terminated ? (statuses)_status : statuses::unknown; }
			
		template<typename types>  struct action_t {
			KALINA1_DRIVE_PMSM_NAME::front::action_t<types> pmsm;
			#if KALINA1_BRAKE_ENABLED == 1
			KALINA1_BRAKE_NAME::front::action_t<types> br;
			#endif
			struct {
				modes mode;
				bool autobrake;
			} controller;
		};
		template<typename types>  struct feedback_t {
			KALINA1_DRIVE_PMSM_NAME::front::feedback_t<types> pmsm;
			#if KALINA1_BRAKE_ENABLED == 1
			KALINA1_BRAKE_NAME::front::feedback_t<types> br;
			#endif
			struct {
				modes mode;
				statuses status;
			} controller;
		};
		struct snapshot0_s {
			enum { mem_size = 8 };
			union mode_s {
				struct {
					uint16_t mode_ix : 6;
					uint16_t control_mode_ix : 3;
					uint16_t fault : 1;
					uint16_t control_status : 2;
					uint16_t reserv : 4;
				};
				uint16_t value;
			};
			uint8_t memo[mem_size];
			template<typename types> void encode(const feedback_t<types>& _feedback) {
				::robo::ostram os(memo, mem_size);
				mode_s m;
				m.fault = _feedback.pmsm.actuator.ps.dev.fault ? 1 : 0;
				m.mode_ix = _feedback.pmsm.actuator.ps.dev.mode;
				m.control_mode_ix = (uint16_t)_feedback.controller.mode;
				m.control_status = (uint16_t)_feedback.controller.status;
				os.put(m.value); //2
				os.put(::mexo::pack< int16_t>(_feedback.pmsm.actuator.ps.current, 0)); //2
				#if KALINA1_MOTOR_ENCO_ENABLED == 1
				os.put(::mexo::pack< int16_t>(_feedback.pmsm.actuator.speed, 0)); //2
				os.put(::mexo::pack< int16_t>(_feedback.pmsm.actuator.position/60, 0)); //2
				#endif
			}
			template<typename types, typename C> void decode(feedback_t<types>& _feedback, const C& _conv) {
				::robo::istram is(memo, mem_size);
				mode_s m;
				is.get(m.value); //2
				_feedback.controller.mode = decode_mode(m.control_mode_ix);
				_feedback.controller.status = decode_status(m.control_status);
				_feedback.pmsm.actuator.ps.dev.mode = m.control_mode_ix;
				_feedback.pmsm.actuator.ps.dev.fault = m.fault != 0;

				{
					int16_t tmp;
					is.get(tmp);
					_feedback.pmsm.actuator.ps.current =
						_conv.current.to_float(tmp);
				}
				{
					int16_t tmp;
					is.get(tmp);
					_feedback.pmsm.actuator.speed = _conv.speed.to_float(tmp);
				}
				{
					int16_t tmp;
					is.get(tmp);
					_feedback.pmsm.actuator.position = _conv.position.to_double(tmp*60);
				}
			}
		};
		struct goal0_s {
			union mode_s {
				struct {
					uint8_t mode_ix : 7;
					uint8_t autobrake : 1;
				};
				uint8_t value;
			};
			template<typename types, typename C> uint8_t encode(const action_t<types>& _action, const C& _conv, uint8_t* _memo, size_t _mem_size) {
				::robo::ostram os(_memo, _mem_size);
				mode_s m;
				m.mode_ix = (uint8_t)_action.controller.mode;
				m.autobrake = _action.controller.autobrake ? 1 : 0;
				os.put(m.value); //1
				switch (_action.controller.mode) {
				case modes::off:
				case modes::unknown:
				case modes::passive:
				case modes::tractor: break;
					#if KALINA1_MOTOR_ENCO_ENABLED == 1
				case modes::position:;
					os.put<int16_t>(_conv.position.to_i16((float)_action.pmsm.actuator.position/60)); //2
				case modes::motion:;
					os.put<int16_t>(_conv.speed.to_i16(_action.pmsm.actuator.speed)); //2
					os.put<int16_t>(_conv.current.to_i16(_action.pmsm.actuator.ps.current)); //2
					#else
				case modes::position:;
				case modes::motion: break;;
					#endif						
				}
				return (uint8_t)os.actual_size();
			}
			template<typename types, typename C> void decode(C& _controller, const uint8_t* _memo, size_t _mem_size) {
				::robo::istram is(_memo, _mem_size);
				mode_s m;
				is.get(m.value);
				switch (decode_mode(m.mode_ix)) {
				case modes::passive:
				_controller.passive_mode();
				break;
				#if KALINA1_MOTOR_ENCO_ENABLED == 1
				case modes::position:;
				{
					int16_t speed, current, position;
					is.get(position);
					is.get(speed);
					is.get(current);
					_controller.position_mode(position*60, speed, current, m.autobrake);
					break;
				}
				case modes::motion:;
					int16_t speed, current;
					is.get(speed);
					is.get(current);
					_controller.speed_mode(speed, current);
					break;
					#endif	
				default:
				_controller.stop();
				break;
				}
			}
		};
	}
}

#endif