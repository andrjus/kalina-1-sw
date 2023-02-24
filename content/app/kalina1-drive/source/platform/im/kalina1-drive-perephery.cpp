#include "im/models/pmsm.hpp"
#include "im/models/gearbox.hpp"
#include "kalina1-drive.hpp"
#include "im/models/pmsm.hpp"
#include "core/robosd_system.hpp"
#include <cstdlib>
#include <cstdio>
#include "net/robosd_serial.hpp"
#include "net/robosd_flow.hpp"
#include "net/robosd_flow_id.h"
#include "freemaster/robosd_fm.hpp"
#include "terminal/robosd_termo.hpp"
#include "core/robosd_log.hpp"
#include "core/robosd_ini.hpp"
#include "im/edev/edev.hpp"
#include "net/platform/can/emulator/emu_can.hpp"
#include "net/platform/serial/win_com.hpp"

namespace kalina1_drive {

	uint32_t periphery::adc::raw[periphery::adc::channel_count];

	#define  _saturate(a, b) (((a) > (b))?(b):(a))

	class agent : public edev::agent {
		robo::net::emu_can::port can_;
		robo::edev::joint::pmsm::ideal2 motor_;
		#if KALINA1_BRAKE_ENABLED == 1
		robo::edev::joint::gearbox::elastic::brake gearbox_;
		#else
		robo::edev::joint::gearbox::elastic::friction gearbox_;
		#endif		
		robo::net::win_com uart0;
		float pwmGain_;
		float voltageSupply_;
		uint8_t address_;
		int currentDriverNoizeBit; //Шум датчика тока относительно драйвера
		int currentDriverMax; //максимальный ток относительно АЦП
		int currentDriverZero; // ноль ток относительно АЦП
		float currentNoizeMag;
		float currentSenceMax;
		float currentGain;

		float armadillo_motorPosition;
		float motorPositionNoizeMag; //rad амплитуда шума на датчике
		int motorDriverNoizePositionBit; //bit  Шум датчика относительно драйвера
		int motorDriverResolution; 
		float motorPositionOffset; // rad смещение датчика относительно ротора в рад
		double motorPositionGain; // pp /rad коэффициент между попугаями драйвера и углом поворота роторашарнира
		int motorPositionRevert;

		#if KALINA1_ROTOR_ENCO_TYPE == KALINA1_ROTOR_ENCO_TYPE_INC
		friend class periphery::enco::rotor::abs;
		int rotorIncPositionRevert;
		uint16_t rotorPosition = 0;
		double rotorPositionGain = (double)((1 << KALINA1_MOTOR_ENCO_ABS_BITS)) / (2 * pi<double>);
		#endif

		struct {
			unsigned int  A;
			unsigned int  B;
			unsigned int  C;
		} pwm;
		struct {
			float  A;
			float  B;
			float  C;
		} ph_voltage;
	public:
		bool power_button_state = false;

		uint32_t motorPosition; // pp
		uint32_t sence_current(float _value) {
			float noize = currentNoizeMag * (float)(rand() % 1000 - 500) / 500.f;
			float tmp = (_value + noize) * currentGain;
			if (tmp > 0)  tmp += 0.5; else if (tmp < 0)	tmp -= 0.5; // округление данных датчика

			int32_t ret = currentDriverZero + (int32_t)tmp;
			if (ret < 0) ret = 0;
			if (ret > currentDriverMax) ret = currentDriverMax;
			return (uint32_t)ret;
		}
		
		void motor_pos_sence(void) {
			//1. Фактическое положение датчика в радианах		
			double armadillo_motorPosition = motor_.actuator.position;// (float)gearbox_.supply.position;

			//2. Датчик получает смещенные данные вместе с шумом. Данные дискретизируются (что особенно важно для измерения скорости)
			//как раз с эффективным разрешением датчика
			//
			double motorPositionNoize = motorPositionNoizeMag * (float)(rand() % 1000 - 500) / 500.f;

			double tmp = motorPositionGain * fmod(armadillo_motorPosition + motorPositionOffset + motorPositionNoize, 2.0f * ::robo::pi<double>);

			if (tmp > 0) {
				tmp += 0.5;
				motorPosition = (uint32_t)(tmp);
			}
			else {
				tmp -= 0.5;
				motorPosition = ((1 << KALINA1_MOTOR_ENCO_ABS_BITS) - 1) - (uint32_t)(-tmp);
			}


			//else if (tmp < 0)	tmp -= 0.5; // округление данных датчика
			if (motorPositionRevert == 1) {
				motorPosition = ((1 << KALINA1_MOTOR_ENCO_ABS_BITS)-1) - motorPosition;
			}
			motorPosition = motorPosition & ((1 << KALINA1_MOTOR_ENCO_ABS_BITS) - 1);
			//Данные датчика положения уже в попугаях	

		}
		
		virtual void perform_command(int _command) {
			power_button_state = _command > 0;
		}

		virtual void do_priotitet_run(double _time){
			static bool odd = false;
			static double last = 0;
			if (_time - last > sample_time) {
				last += sample_time;
				float pwmA = (float)pwm.A;
				float pwmB = (float)pwm.B;
				float pwmC = (float)pwm.C;
				ph_voltage.A = (pwmA - (pwmB + pwmC) / 2) / pwmGain_;
				ph_voltage.B = (pwmB - (pwmA + pwmC) / 2) / pwmGain_;
				ph_voltage.C = (pwmC - (pwmA + pwmB) / 2) / pwmGain_;; // таже самая формула, только считает быстрее
				motor_.set_phase_voltage(ph_voltage.A, ph_voltage.B, ph_voltage.C);

				periphery::adc::raw[0] = sence_current(motor_.current.phase.A);
				periphery::adc::raw[1] = sence_current(motor_.current.phase.B);
				periphery::adc::raw[2] = sence_current(motor_.current.phase.C);
				#if KALINA1_BRAKE_ENABLED == 1
				periphery::adc::raw[3] = sence_current(br_.current * 60); //todo измирение тока
				#endif
				motor_pos_sence();
				::mexo::machine::realtime_loop();
				::mexo::machine::backend_loop();
				time_ = _time;
			}
			#if KALINA1_BRAKE_ENABLED == 1
			br_.run();
			if (br_.fixed()) {
				gearbox_.set();
			}
			else {
				gearbox_.release();
			}
			#endif
		}
		virtual void do_background_run(double _time){
			::mexo::machine::frontend_loop();
			can_.poll();
		}
		void send(unsigned _id, const uint8_t* _data, uint8_t _size) {
			if (can_.ready()) {
				can_.send(_id, _data, _size);			
			}
		}

		#ifdef ENV_NET_FLOW_PORT_PATH
		friend class can_port_driver;
		class can_port_driver {
		public:
			typedef flow_msg_can_id_t id_t;
			enum { suba_count = 16, packet_size = 8, msg_pool_size = 4 };
			static inline cstr path = ENV_NET_FLOW_PORT_PATH;
			static void send(uint32_t _id, const uint8_t* _data, uint8_t _size);
		};
		::robo::net::flow::port_t<can_port_driver> can0;
		#endif

		//delegat::srmember<agent, void, ::robo::net::ican&, uint32_t, const uint8_t*, uint8_t   > 
		delegat::owned_fabric<void, ::robo::net::ican&, uint32_t, const uint8_t*, uint8_t   >::member<agent> on_can_receive_;
		void on_can_receive__(::robo::net::ican& _ican, uint32_t _id, const uint8_t* _data, uint8_t _len) {
			#ifdef ENV_NET_FLOW_PORT_PATH			
			if ( ( ((_id & 0xF0)>>4) == address_) || ((_id & 0xF0) == 0) ) {
				can0.on_receive(
					_id
					, _data
					, _len
				);
			}
			#endif
		}
		virtual bool do_begin(void) {
			ROBO_LBREAKN(edev::agent::do_begin());
			::robo::time_us_t tmp = ((::robo::time_us_t)((edev::agent::sample_time * 1000000.)));
			ROBO_LBREAKN_F(tmp  == KALINA1_PWM_PERIOD_US,\
						   RT("the static time does not match the one loaded from the ini - static: %d, loaded: %d"), KALINA1_PWM_PERIOD_US, tmp );
			ROBO_LBREAKN(robo::ini::load(name, RT("ADDRESS"), address_));

			ROBO_LBREAKN(robo::ini::load(name,type, RT("CHANNEL"), can_.channel));
			ROBO_LBREAKN(robo::ini::load(name, type, RT("REPEAT_MAX_COUNT"), can_.repeat_max_count));

			ROBO_LBREAKN(robo::ini::load(name, type, RT("supply_voltage"), voltageSupply_) );
			ROBO_LBREAKN((voltageSupply_ > 0.f) && (KALINA1_PWM_MODULO > 0));// && (pwmMax_ > 0) && (pwmModulo_ > pwmMax_) && (pwmMax_ > pwmMin_))
			pwmGain_ = KALINA1_PWM_MODULO / voltageSupply_;


			ROBO_LBREAKN(robo::ini::load(name, type, RT("currentSenceMax"), currentSenceMax));

			currentDriverMax = (1 << KALINA1_ADC_BITS) - 1;
			currentDriverZero = currentDriverMax >> 1;
			currentGain = (float)currentDriverMax / (2.0f * currentSenceMax);
			currentNoizeMag = (2.0f * currentSenceMax) / (1 << (KALINA1_ADC_BITS - currentDriverNoizeBit));

			ROBO_LBREAKN(robo::ini::load(name, type, RT("motorDriverNoizePositionBit"), motorDriverNoizePositionBit) );
			ROBO_LBREAKN(robo::ini::load(name, type, RT("motorPositionRevert"), motorPositionRevert) );
			motorPositionNoizeMag = (2.0f * ::robo::pi<float> ) / (1 << (KALINA1_MOTOR_ENCO_ABS_BITS - motorDriverNoizePositionBit));
			
			if (motorDriverNoizePositionBit == 0) {
				motorPositionNoizeMag = 0.0;
			}
			else {
				motorPositionNoizeMag = (2.0f * ::robo::pi<float>) / (1 << (KALINA1_MOTOR_ENCO_ABS_BITS)) * (1 << (motorDriverNoizePositionBit - 1));
			}
			ROBO_LBREAKN(robo::ini::load(name, type, RT("motorPositionOffset"), motorPositionOffset) );

			motorDriverResolution = (1 << KALINA1_MOTOR_ENCO_ABS_BITS);
			motorPositionGain = (float)(motorDriverResolution) / (2.0f * ::robo::pi<float>);
			
			#if KALINA1_BRAKE_ENABLED == 1
			ROBO_LBREAKN(br_.load(name, type));
			#endif 

			can_.open();
			
			can_.set_on_receive(&on_can_receive_);

			string comm;
			if (comm.tryload(name, RT("UART0"))) {
				uart0.connect(comm);
				uart0.begin(RT("UART0"));
			}

			::mexo::machine::begin();
			::mexo::machine::start();
			uint8_t addr = 0;
			return true;
		}
		virtual void do_reconfig(void) {
		}
		virtual void do_finish(void) {
			can_.close();
		}
		virtual void set_local_ini(cstr _ini) { system::ini::begin(_ini); }
	
		double time_=0.0;
		agent(void) :
			motor_(*this, RT("motor"))
			, gearbox_(*this, RT("load"))
			, on_can_receive_(*this, &agent::on_can_receive__)
		{
			
			::mexo::var::record::create(::mexo::var::types::const_uint16, time_, RT("im.tm"));
			::mexo::var::record::create(::mexo::var::types::const_uint16, pwm.A, RT("im.pwm.A"));
			::mexo::var::record::create(::mexo::var::types::const_uint16, pwm.B, RT("im.pwm.B"));
			::mexo::var::record::create(::mexo::var::types::const_uint16, pwm.C, RT("im.pwm.C"));

			::mexo::var::record::create(::mexo::var::types::const_real, ph_voltage.A, RT("im.v.A"));
			::mexo::var::record::create(::mexo::var::types::const_real, ph_voltage.B, RT("im.v.B"));
			::mexo::var::record::create(::mexo::var::types::const_real, ph_voltage.C, RT("im.v.C"));

			::mexo::var::record::create(::mexo::var::types::const_real, motor_.voltage.ab.alfa, RT("im.v.alfa"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.voltage.ab.beta, RT("im.v.beta"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.voltage.dq.d, RT("im.v.d"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.voltage.dq.q, RT("im.v.q"));

			::mexo::var::record::create(::mexo::var::types::const_real, motor_.current.ab.alfa, RT("im.c.alfa"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.current.ab.beta, RT("im.c.beta"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.current.dq.d, RT("im.c.d"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.current.dq.q, RT("im.c.q"));

			::mexo::var::record::create(::mexo::var::types::const_real, motor_.actuator.speed, RT("im.c.mo.sp"));
			::mexo::var::record::create(::mexo::var::types::const_ext, motor_.actuator.position, RT("im.c.mo.po"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.actuator.driveng_torque, RT("im.c.mo.torque"));
			::mexo::var::record::create(::mexo::var::types::const_real, motor_.actuator.contr_torque, RT("im.c.mo.load"));
			::mexo::var::record::create(::mexo::var::types::const_uint8, motor_.actuator.state, RT("im.c.mo.state"));
			::mexo::var::record::create(::mexo::var::types::const_real, gearbox_.driver.speed, RT("im.gb.sp"));
			::mexo::var::record::create(::mexo::var::types::const_ext, gearbox_.driver.position, RT("im.gb.po"));

			gearbox_.connect_to_actuator(&motor_.actuator);
		}
		void pwm_do_run(const periphery::pwm::inverter::duty_t& _duty) {
			pwm.A = _duty.A;
			pwm.B = _duty.B;
			pwm.C = _duty.C;
		}
		void pwm_boot_complete(const periphery::pwm::inverter::duty_t& _duty) {
			pwm.A = _duty.A;
			pwm.B = _duty.B;
			pwm.C = _duty.C;
			motor_.powerOn = true;
		}

		void pwm_shutdown_begin(void) {
			motor_.powerOn = false;			
		}
		#if KALINA1_BRAKE_ENABLED == 1
		class br {
			float R = 1.f;
			float L = 0.f;
			float currentHi = 0.f;
			float currentLo = 0.f;
			float ps_voltage = 0.f;
			float tmeout_ms = 1.f;
			float voltage=0.f;
			float pwm= 0.f;
			bool powerOn=false;
			double A = 1.f;
			double currentd = 0.;
			float pwmGain = 1.f;
			float currentMax = 0.f;
			float position = 0.f;
			float speed = 0.f;
		public:
			float current = 0.f;
			bool load (cstr _name, cstr _type) {
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.R"), R));
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.L"), L));
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.currentHi"), currentHi));
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.currentLo"), currentLo));
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.ps_voltage"), ps_voltage));
				ROBO_LBREAKN(robo::ini::load(_name, _type, RT("br.tmeout_ms"), tmeout_ms));
				A = exp(-R /L* 0.000001 * KALINA1_PWM_PERIOD_US);
				speed = (float)(1000. / tmeout_ms* 0.000001 * KALINA1_PWM_PERIOD_US);
				pwmGain = ps_voltage/ KALINA1_PWM_MODULO;

				::mexo::var::record::create(::mexo::var::types::const_real, voltage, RT("im.br.v"));
				::mexo::var::record::create(::mexo::var::types::const_real, current, RT("im.br.c"));
				::mexo::var::record::create(::mexo::var::types::const_real, position, RT("im.br.po"));


				return true;
			}
			bool fixed(void) { return position < 1.f; }
			void run(void) {
				if (powerOn) {
					voltage = pwm * pwmGain;
					currentMax = voltage /R;
					currentd =  - A * (currentMax - currentd) + currentMax;
				}
				else {
					voltage = 0.f;
					currentd=   A *  current;
				}
				current = (float)currentd;
				if (current < currentHi && current > currentLo) {
					if (position < 1.f) {
						position += speed;
					}
				}
				else {
					if (position > 0.f) {
						position -= speed;
					}
				}
			}
			void pwm_do_run(const periphery::br_pwm::inverter::duty_t& _duty) {
				pwm = _duty;
			}

			void pwm_boot_complete(const periphery::br_pwm::inverter::duty_t& _duty) {
				pwm = _duty;
				powerOn = true;
			}
			
			void pwm_shutdown_begin(void) {
				powerOn = false;
			}

		} br_;
		#endif
	} agent_;
	//todo хорошобы драйвер сделать с конструктором, имеющим параметрами
	void agent::can_port_driver::send(uint32_t _id, const uint8_t* _data, uint8_t _size) {
		agent_.can_.send(_id, _data, _size);
	}

}

extern "C" {
	ROBO_EXPORT_RUNTIME robo::edev::agent* ROBO_EXPORT_RUNTIME_DECL query_agent(void) {
		return &kalina1_drive::agent_;
	}
}

#include "mexo/mexo.hpp"
//uint32_t mexo_drive::current_adc_driver::sence[2] = { 0,0 };

namespace kalina1_drive {

	void periphery::pwm::boot_complete(const inverter::duty_t& _duty) {
		agent_.pwm_boot_complete(_duty);
	}

	void periphery::pwm::shutdown_begin(void) {
		agent_.pwm_shutdown_begin();
	}

	void periphery::pwm::do_run(const inverter::duty_t& _duty) {
		agent_.pwm_do_run(_duty);
	}

	void periphery::pwm::boot_begin(void) {
	}
	bool periphery::pwm::do_boot(void) {
		return true;
	}
	bool periphery::pwm::do_shutdown(void) {
		return true;
	}
	void periphery::pwm::shutdown_complete(void) {
	}

	bool periphery::power_button_state(void) {
		return agent_.power_button_state;
	}
	#if KALINA1_BRAKE_ENABLED == 1
	void periphery::br_pwm::boot_complete(const inverter::duty_t& _duty) {
		agent_.br_.pwm_boot_complete(_duty);
	}

	void periphery::br_pwm::shutdown_begin(void) {
		agent_.br_.pwm_shutdown_begin();
	}

	void periphery::br_pwm::do_run(const inverter::duty_t& _duty) {
		agent_.br_.pwm_do_run(_duty);
	}
	void periphery::br_pwm::boot_begin(void) {}
	bool periphery::br_pwm::do_boot(void) {
		return true;
	}
	bool periphery::br_pwm::do_shutdown(void) {
		return true;
	}
	void periphery::br_pwm::shutdown_complete(void) {
	}
	#endif

	void periphery::adc::query(void) {}


	#if KALINA1_ROTOR_INC_ENABLED == 1
	//драйвер  энкодера положения ротора на инкрементном датчике см "mexo/enco.hpp
	void	periphery::enco::rotor::increment::begin(void) {
		//double armadillo_motorPosition = motor_.actuator.position
		agent_.rotorIncPosition_prev =agent_.motor_.actuator.position;
		agent_.rotorIncPosition = 0;
	}
	uint32_t	periphery::enco::rotor::increment::encode(void) {
		return agent_.rotorIncPosition;
	}
	void	periphery::enco::rotor::increment::query(void) {
		agent_.rotorIncPosition = (uint16_t)( (agent_.motor_.actuator.position - agent_.rotorIncPosition_prev) * agent_.rotorIncPositionGain);
	}
	bool 	periphery::enco::rotor::increment::error(void) {
		return false;
	}
	#endif

	#if KALINA1_ROTOR_ABS_ENABLED == 1
	uint32_t  BiSS_driver::encode(void) {
		return agent_.motorPosition;
	}

	bool  BiSS_driver::error(void) {
		return false;
	}

	void BiSS_driver::query(void) {
		agent_.motor_pos_query();
	}
	#endif

	uint16_t  periphery::enco::rotor::abs::encode(void) {
		return agent_.motorPosition;
	}


}

IMAGE_DOS_HEADER __ImageBase;
uintptr_t MODULE_ADDRESS = (uintptr_t)&__ImageBase;



