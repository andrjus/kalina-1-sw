#include "kalina1-drive.hpp"
#include "net/robosd_proto_switch.hpp"
#include "mexo/mexo_brake.hpp"
#include "core/robosd_ring_buf.hpp"


namespace kalina1_drive {

	::mexo::led<periphery::tp2> tp2;

	//1.1	Набор блоков, обеспечивающий взаимодействие переферии и системы управления

	// подсистема постоянно работает в контексте prioritet ( будет срабатывать по прерыванию от АЦП)
	::mexo::realtime_subsystem prioritet_subsystem(RT("prior"), false);
	
	// подсистема работает каждый второй (из четырех) слотов
	::mexo::periodic_subsystem periodic_subsystem(RT("loop"), false, { 1 });
	
	// подсистема постоянно работает в контексте backend
	::mexo::backend_subsystem backend_subsystem(RT("back"), false);

	struct pmsm_hardware {
		::mexo::realtime_subsystem& realtime_subsystem;
		::mexo::periodic_subsystem& periodic_subsystem;
		::mexo::backend_subsystem& backend_subsystem;
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		motor_enco motor_enco_block;
		#endif
		rotator rotator_block;
		current_abc_sensor current_sence_block;
		abc_power_supply power_supply_block;
		void reconfig(void) { power_supply_block.reconfig(); };
		pmsm_hardware(void);
		~pmsm_hardware(void) {}
		#if KALINA1_MOTOR_ENCO_ENABLED != 1
		uint32_t enco_dummy;
		#endif
	} pmsm_hardware_;
	
	#if KALINA1_BRAKE_ENABLED == 1
	struct br_hardware {
		::mexo::realtime_subsystem& realtime_subsystem;
		::mexo::backend_subsystem& backend_subsystem;
		br_power_supply power_supply_block;
		br_current_sensor current_sence_block;
		br_hardware(
			::mexo::realtime_subsystem& _prioritet_subsystem
			, ::mexo::backend_subsystem& _backend_subsystem
		);
		~br_hardware(void) {}
		void reconfig(void) { power_supply_block.reconfig(); };
	} br_hardware_(pmsm_hardware_.realtime_subsystem, pmsm_hardware_.backend_subsystem);
	#endif

	//1.2 Сам актуатор
	typedef ::pmsm::dev_t<types, pmsm_hardware>  pmsm_t;

	//тормоз
	#if KALINA1_BRAKE_ENABLED == 1
	typedef ::br::dev_t<types, br_hardware>  br_t;
	#endif

	//2. структуры с конфигурационными данными  и текцщими значенниями сигналов
	//2.1. тут храним все актуальные сигналы в одном месте
	struct {
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		motor_enco::present_s motor_enco;
		#endif
		rotator::present_s rotator;
		abc_power_supply::present_s abc_power_supply;
		current_abc_sensor::present_s current_sensor;
		pmsm_t::present_s pmsm;
		#if KALINA1_BRAKE_ENABLED == 1
		br_power_supply::present_s br_power_supply;
		br_current_sensor::present_s br_current_sensor;
		br_t::present_s br;
		#endif
	} present = {};

	//2.2. тут храним все настройки в одном месте
	struct {
		abc_power_supply::config_s abc_power_supply;
		current_abc_sensor::config_s current_sensor;
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		motor_enco::config_s motor_enco;
		#endif
		rotator::config_s rotator;
		pmsm_t::config_s pmsm;
		#if KALINA1_BRAKE_ENABLED == 1
		br_power_supply::config_s br_power_supply;
		br_current_sensor::config_s br_current_sensor;
		br_t::config_s br;
		#endif
		struct {
			struct {
				::robo::time_us_t prepare_us;
				::robo::time_us_t relax_us;
				::robo::time_us_t shutdown_us;
				int auotostop_tk;
				::robo::time_us_t poll_us;
			} pause;
			#if KALINA1_POSITION_SENCE_ENABLED == 1
			struct {
				int speed;
				int position;
			} mode;
			#endif
			types::signal_t current_max;
			bool autobrake;
			bool payload_enabled;
			::robo::time_us_t payload_start_pause_us;
			types::signal_t payload_speed_hi;
			types::signal_t payload_speed_lo;

		} controller;

	} config =
	{
		{//abc_power_supply
			{0}
			,{
				KALINA1_PWM_MIN
				,KALINA1_PWM_MAX
			}
			,{
				-KALINA1_VOLTAGE_PP_MAX
				,KALINA1_VOLTAGE_PP_MAX
			}
		}
		,
		{//current_sensor
			{0}
			,{KALINA1_CURRENT_IX_A,KALINA1_CURRENT_IX_B,KALINA1_CURRENT_IX_C}
			,{1,1,1}
			,KALINA1_ADC_SETUP_PERIOD_BITS
		}
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		,{// motor_enco
			{0}
			, 10
		}
		#endif

		,{//rotator
			{0}
			,KALINA1_ROTOR_OFFSET
			,KALINA1_ROTOR_REVERT
			, KALINA1_ROTOR_POLE_COUNT
		}
		,{} //pmsm - вписывает сам при инициализации в статическом конструкторе -- говнокод!
		#if KALINA1_BRAKE_ENABLED == 1
		,{ //br_power_supply
			{ 0 }
			,{
				KALINA1_BRK_PWM_MIN
				,KALINA1_PWM_MAX
			}
			,{
				0
				,KALINA1_VOLTAGE_PP_MAX
			}
		}
		, {} //br_current_sensor
		, {} //br - вписывает сам при инициализации в статическом конструкторе -- говнокод!
		#endif
		,{
			{
				1000//	::robo::time_us_t prepare_us = 1000;
				,3000 //	::robo::time_us_t relax_us = 3000;
				,100000//	::robo::time_us_t shutdown_us = 100000;
				,1000//	int auotostop_tk = 1000;
				,20000//	::robo::time_us_t poll_us = 20000;
			}// pause;
			#if KALINA1_POSITION_SENCE_ENABLED == 1
			,{
				pmsm::front::mode::speed_ov_current
				,pmsm::front::mode::position_ov_current
			} //mode;
			#endif
			,KALINA1_CURRENT_MAX_PP // types::signal_t current_max = KALINA1_CURRENT_MAX_PP;
			, false //bool autobrake = false;
			,false //bool payload_enabled = false;
			,10000 //::robo::time_us_t payload_start_pause_us = 10000;
			,10 //types::signal_t payload_speed_hi = 10;
			,5//types::signal_t payload_speed_lo = 5;

		} //controller;

	};

	//2.3. тут храним все сигналы управления в одном месте
	front::action_t<types> action = { };
	//2.3. тут храним все сигналы обратной связи
	front::feedback_t<types> feedback = { };

	//3 конструктор соединяет блоки в порядке, который обеспечить строго заданную последовательность 
	//выполнения
	pmsm_hardware::pmsm_hardware(void) :
		//4.конфигурируем подсистему backend
		// подсистема - работает в контексте backend и может быть отложено на несколько микросекунд
		 backend_subsystem(::kalina1_drive::backend_subsystem)
		, realtime_subsystem(::kalina1_drive::prioritet_subsystem)
		// еще один системный сервис - будет работать каждый четвертый период - в слоте 1 (общий для всех)
		, periodic_subsystem(::kalina1_drive::periodic_subsystem)
		//3.1. конфигурируем приоритетную подсистему 
		//3.1. подключаем датчик положения мотора - он первый, так как определяет угол поворота 
		//системы координат dq
	//	, motor_enco_block(RT("mo_enco"), &prioritet_subsystem, periphery_config.motor_enco, present.motor_enco)
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		//motor_enco_block - сделан как отдельная подсистема, 
		, motor_enco_block(RT("mo_enco"), &prioritet_subsystem, config.motor_enco, present.motor_enco)
		#endif
		//3.2. подключаем ротатор- расчмитывает син и кос для матрицы поворота
		//подключаемся к тойже подсистеме , к которой подключен motor_enco_block, сразу после него 	
		//, rotator_block(RT("enco_conv"), &motor_enco_block, periphery_config.rotator, present.rotator, present.motor_enco.native.ceiled)
		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		, rotator_block(RT("enco_conv"), &prioritet_subsystem, config.rotator, present.rotator, present.motor_enco.native.ceiled)
		#endif
		#if KALINA1_MOTOR_ENCO_TYPE == KALINA1_MOTOR_ENCO_TYPE_NONE
		, rotator_block(RT("rotator"), &prioritet_subsystem, config.rotator, present.rotator, enco_dummy)
		#endif
		//3.2.  подключаем к подсистеме датчик тока current_sence_block
		, current_sence_block(RT("sence.c"), &rotator_block, config.current_sensor, present.current_sensor, present.rotator.fb.output)
		//4.1. подключаем к hardware backend системе силовой преобразователь
		//  - он будет срабатывать последним и подготовит сигналы управления
		, power_supply_block(RT("ps"), &backend_subsystem, config.abc_power_supply, present.abc_power_supply, present.rotator.fb.output)
	{
		static ::mexo::machine::slot::simple start(
			::mexo::machine::slot::kind::start
			, [] {
				pmsm_hardware_.realtime_subsystem.start();									//активируем подсистему аппаратуры
				pmsm_hardware_.backend_subsystem.start();									//активируем подсистему аппаратуры
				pmsm_hardware_.periodic_subsystem.start();
				#if KALINA1_MOTOR_ENCO_ENABLED == 1
				pmsm_hardware_.motor_enco_block.start();
				#endif
			});

		#if KALINA1_MOTOR_ENCO_ENABLED == 1
		//так часто-для ротатора
		motor_enco_block.setup({0,1,2,3});
		#endif
	}

	//4. сам моторчик +1мкс - инфраструкутура и быстрый фильтр  +1мкс - напряженческий, +2,5мкс -токовый
	// фильтры, контура скорости, тока и др
	pmsm_t pmsm_(pmsm_hardware_, RT("pmsm"), action.pmsm, feedback.pmsm, config.pmsm, present.pmsm, 0);


	//5. тормоз
	#if KALINA1_BRAKE_ENABLED == 1
	br_t br(br_hardware_, RT("br"), action.br.ps, feedback.br.ps, config.br, present.br, 1);
	#endif


	#if KALINA1_BRAKE_ENABLED == 1
	br_hardware::br_hardware(
		::mexo::realtime_subsystem& _realtime_subsystem
		, ::mexo::backend_subsystem& _backend_subsystem
	) :
		realtime_subsystem(_realtime_subsystem)
		, backend_subsystem(_backend_subsystem)
		, power_supply_block(RT("brp"), &backend_subsystem, config.br_power_supply, present.br_power_supply)
		, current_sence_block(RT("sence.br.c"), &backend_subsystem, config.br_current_sensor, present.br_current_sensor) {
	};
	#endif

	//8. делегат в слот "begin" - сработает при инициализации - собираем все вместе
	::mexo::machine::slot::simple begin(
		::mexo::machine::slot::kind::begin
		, [] {
			::mexo::tp.set_verb(::mexo::tp_verb::loop);
		}
	);

	//контроллер

	class controller : public ::mexo::controller {
		#if KALINA1_PAYLOAD_ENABLED
		::robo::time_us_t payload_begin_tm_us_ = 0;
		enum class payload_status { off, pause, on } payload_status_ = payload_status::off;
		#endif
	public:
		/*struct {
			types::signal_t speed_max = 0;
			types::signal_t speed = 0;
			struct {
				types::long_signal_t value = 0;
				types::long_signal_t min = KALINA1_ACTUATOR_ENCO_MIN;
				types::long_signal_t max = KALINA1_ACTUATOR_ENCO_MAX;
			} position;
		} req;*/

		/*struct action{
			types::signal_t position;
			types::signal_t speed;
			types::signal_t current;
			bool autobrake;

		};*/


		typedef kalina1_drive::front::modes modes;
		typedef kalina1_drive::front::statuses statuses;
		bool update_need = false;
		#if KALINA1_PAYLOAD_ENABLED
		void payload_on(void) {
			tp2.on();
			payload_status_ = payload_status::on;
		}
		void payload_off(void) {
			tp2.off();
			payload_status_ = payload_status::off;
		}
		void check_payload(void) {
			if (config.controller.payload_enabled) {
				switch (payload_status_) {
				case payload_status::off:
				if (abs(present.pmsm.speed_filter.fb.output) > config.controller.payload_speed_hi) {
					payload_begin_tm_us_ = ::robo::system::env::time_us();
					payload_status_ = payload_status::pause;
				}
				break;
				case payload_status::pause:
				if (abs(present.pmsm.speed_filter.fb.output) < config.controller.payload_speed_lo) {
					payload_off();
				}
				else {
					if ((::robo::system::env::time_us() - payload_begin_tm_us_) > config.controller.payload_start_pause_us) {
						payload_on();
					}
				}
				break;
				case payload_status::on:
				if (abs(present.pmsm.speed_filter.fb.output) < config.controller.payload_speed_lo) {
					payload_off();
				}
				break;
				}
			}
		}
		#endif
	private:
		modes mode_ = modes::off;
		statuses status_ = statuses::none;
		friend class passive_task;
		//при запуске - снимаемся с тормоза и считаем, что перешли в штатный режим после фактического снятия с тормоза
		class passive_task : public process {
		protected:
			controller& controller_;

			virtual void onStartup(void) {
				#if KALINA1_BRAKE_ENABLED == 1
				present.br.command = ::mexo::brake::itf::command_t::release;
				#endif
			}
			virtual result doStartup(void) {
				#if KALINA1_BRAKE_ENABLED == 1
				return present.br.status == ::mexo::brake::itf::status_t::releasing ? result::success : result::wait;
				#else
				return result::success;
				#endif
			}
			virtual void onShutdown(void) {
				#if KALINA1_BRAKE_ENABLED == 1
				present.br.command = ::mexo::brake::itf::command_t::set;
				#endif
			}
			virtual result doShutdown(void) {
				#if KALINA1_BRAKE_ENABLED == 1
				return present.br.status == ::mexo::brake::itf::status_t::fixed ? result::success : result::wait;
				#else
				return result::success;
				#endif
			}
			virtual void onFinish(void) {}

		public:
			passive_task(controller& _controller) : process(), controller_(_controller) {}
			virtual ~passive_task(void) {};
		};

		#if KALINA1_POSITION_SENCE_ENABLED == 1

		//сначала выдерживаем паузу 
		// только потом снимаемся с тормоза 
		//и считаем, что перешли в штатный режим после фактического снятия с тормоза
		// Перед постановкой на тормоз ждем, пока привод успокоится
		class move_task : public passive_task {
		protected:
			::robo::time_us_t last_tm = 0;
			bool wait_brake_ = false;
			virtual void onPrepare(void) {
				passive_task::onPrepare();
				last_tm = ::robo::system::time_us();
				#if KALINA1_PAYLOAD_ENABLED				
				controller_.payload_off();
				#endif
			}
			virtual result doPrepare(void) {
				if ((::robo::system::time_us() - last_tm) > config.controller.pause.prepare_us) {
					return result::success;
				}
				else {
					return result::wait;
				}

			}
			void onShutdown(void) {
				last_tm = ::robo::system::time_us();
				#if KALINA1_PAYLOAD_ENABLED				
				controller_.payload_off();
				#endif
			}

			//в процессе остановки даем сначала успокоитьс
			result doShutdown(void) {
				if (wait_brake_) {
					return passive_task::doShutdown();
				}
				else {
					if ((abs(feedback.pmsm.actuator.speed) <= feedback.pmsm.actuator.crawl_speed)
						||
						((::robo::system::time_us() - last_tm) > config.controller.pause.shutdown_us)) {
						action.pmsm.actuator.speed = 0;
						wait_brake_ = true;
						passive_task::onShutdown();
					}
					return result::wait;
				}
			}

			//после постановки на тормоз выдерживаем паузу, а после этого выключаем питание

			virtual void onRelax(void) {
				wait_brake_ = false;
				last_tm = ::robo::system::time_us();
			}
			virtual result doRelax(void) {
				return (::robo::system::time_us() - last_tm) > config.controller.pause.relax_us ? result::success : result::wait;;
			}
			virtual void onFinish(void) {
				action.pmsm.actuator.ps.dev.mode = pmsm::front::mode::idle;
				passive_task::onFinish();
			}
		public:
			move_task(controller& _controller) : passive_task(_controller) {}
			virtual ~move_task(void) {};
		};

		class motion : public move_task {
			friend class controller;
			types::signal_t speed_ = 0;
			types::signal_t speed_prev_ = 0;
			types::signal_t current_ = 0;
			types::long_signal_t fixed_position_ = 0;
			
			pmsm::front::profil_t<types> profil_ = {};
			motion(controller& _controller) : move_task(_controller) {}
			virtual void onPrepare(void) {
				if (config.pmsm.actuator.ps.invers) {
					action.pmsm.actuator.position = -feedback.pmsm.actuator.position;
				}
				else {
					action.pmsm.actuator.position = feedback.pmsm.actuator.position;
				}
				action.pmsm.actuator.speed = 0;
				action.pmsm.actuator.ps.voltage = config.abc_power_supply.voltage.hi;
				action.pmsm.actuator.ps.current = current_;
				action.pmsm.actuator.ps.dev.mode = config.controller.mode.speed;
				fixed_position_ = feedback.pmsm.actuator.position;
				present.pmsm.actuator.ps.dev.action_actual = true;
				controller_.update_need = true;
			}

			virtual result doPrepare(void) {
				profil_.actuator = pmsm_.profil(config.controller.mode.speed);
				return move_task::doPrepare();
			}


			virtual result doExecute(void) {
				#if KALINA1_PAYLOAD_ENABLED				
				controller_.check_payload();
				#endif
				if (controller_.update_need) {
					action.pmsm.actuator.ps.current = current_;
					if (speed_ == 0) {
						if (speed_prev_ != 0) {
							fixed_position_ = feedback.pmsm.actuator.position;
						}
						types::long_signal_t err = fixed_position_ - feedback.pmsm.actuator.position;
						if (err < -profil_.actuator.dead_zone) {
							action.pmsm.actuator.speed = profil_.actuator.crawl_speed;
						}
						else {
							if (err > profil_.actuator.dead_zone) {
								action.pmsm.actuator.speed = -profil_.actuator.crawl_speed;
							}
							else {
								action.pmsm.actuator.speed = 0;
							}
						}
					}
					else {
						action.pmsm.actuator.speed = speed_;
					}

					present.pmsm.actuator.ps.dev.action_actual = true;
					controller_.update_need = false;

				}
				return result::wait;
			}
			virtual void onShutdown(void) {
				action.pmsm.actuator.speed = 0;
				present.pmsm.actuator.ps.dev.action_actual = true;
				move_task::onShutdown();
			}
		};


		class posicioner : public move_task {
			friend class controller;
			int auto_stop_counter_ = 0;
			types::signal_t current_ = 0;
			types::signal_t speed_ = 0;
			types::long_signal_t position_ = 0;

		protected:
			virtual void onPrepare(void) {
				if (config.pmsm.actuator.ps.invers) {
					action.pmsm.actuator.position = -feedback.pmsm.actuator.position;
				}
				else {
					action.pmsm.actuator.position = feedback.pmsm.actuator.position;
				}
				action.pmsm.actuator.speed = feedback.pmsm.actuator.crawl_speed;
				action.pmsm.actuator.ps.voltage = config.abc_power_supply.voltage.hi;
				action.pmsm.actuator.ps.current = current_;
				action.pmsm.actuator.ps.dev.mode = config.controller.mode.position;
				present.pmsm.actuator.ps.dev.action_actual = true;
				controller_.update_need = true;
				auto_stop_counter_ = config.controller.pause.auotostop_tk;
				move_task::onPrepare();
			}


			virtual result doExecute(void) {
				#if KALINA1_PAYLOAD_ENABLED
				controller_.check_payload();
				#endif
				if (controller_.update_need) {
					action.pmsm.actuator.ps.current = current_;
					action.pmsm.actuator.speed = speed_;
					action.pmsm.actuator.position = position_;
					present.pmsm.actuator.ps.dev.action_actual = true;
					controller_.update_need = false;
				}
				else {
					if (config.controller.autobrake && present.pmsm.actuator.speed_deseired == 0) {
						if (auto_stop_counter_ > 0) {
							auto_stop_counter_--;
						}
						else {
							stop();
						}
					}
				}
				return result::wait;
			}

			virtual void onShutdown(void) {
				if (config.pmsm.actuator.ps.invers) {
					action.pmsm.actuator.position = -feedback.pmsm.actuator.position;
				}
				else {
					action.pmsm.actuator.position = feedback.pmsm.actuator.position;
				}
				present.pmsm.actuator.ps.dev.action_actual = true;
				move_task::onShutdown();
			}
		public:
			posicioner(controller& _controller) : move_task(_controller) {}
		};

		#endif


		#if KALINA1_TRACTOR_ENABLED == 1
		class tractor : public posicioner {
			friend class controller;
		protected:
			virtual void onBegin(void) {
				action.pmsm.pmsm.position = present.motor_enco.position;
				action.pmsm.pmsm.speed = 0;
				action.pmsm.pmsm.ps.voltage = pmsm_hardware_.power_supply_block.pwm_voltage_limits().hi;
				action.pmsm.pmsm.ps.current = controller_.settings.current_max;
				action.pmsm.pmsm.ps.dev.mode = controller_.settings.position_mode;
				action.pmsm.pmsm.ps.dev.actual = true;
				move_task::onBegin();
			}
			virtual void onExecute(void) {
				action.pmsm.pmsm.speed = controller_.settings.GLimVel;
				action.pmsm.pmsm.ps.dev.actual = true;
			}
			virtual void onShutdown(void) {
				present.pmsm.pmsm.speed_force = 0;
				posicioner::onShutdown();
			}

			virtual bool doExecute(void) {
				return false;
			}

		private:
			struct point {
				types::long_signal_t position;
				types::signal_t speed;
				uint32_t tm;
			};
			::robo::struct_ring_t<4, point> path_;
			int64_t Cs1 = 0;
			int64_t As2 = 0;
			int64_t Bs2 = 0;
			int64_t As1 = 0;
			int64_t Bs1 = 0;
			int64_t T1 = 0;
			int64_t	T2 = 0;
			int64_t	T3 = 0;
			int64_t tau = 0;
			enum { sh = 25 };
			point p0;
			point p1;
			int64_t delta = 0;
			void approxx_(void) {
				T1 = p1.tm - p0.tm;
				T2 = T1 * T1;
				T3 = T2 * T1;
				delta = p0.position - p1.position;
				As1 = ((2LL * delta + (((T1) * ((int64_t)p0.speed + p1.speed)) >> 4))) << sh;
				if (As1 > 0) {
					As1 = (As1 + (T3 >> 1)) / T3;
				}
				else {
					As1 = -(-As1 + (T3 >> 1)) / T3;
				}

				Bs1 = (-(3LL * delta + (((T1) * (2LL * p0.speed + p1.speed)) >> 4))) << sh;

				if (Bs1 > 0) {
					Bs1 = (Bs1 + (T2 >> 1)) / T2;
				}
				else {
					Bs1 = -(-Bs1 + (T2 >> 1)) / T2;
				}

				Cs1 = ((int64_t)p0.speed) << (sh - 4);
				As2 = As1 * 48;
				Bs2 = Bs1 * 32;
				tau = 0;
			}
			void interp(void) {
				tmp_space = path_.space();
				if (tau < T1) {
					tau++;
					int64_t tau2 = tau * tau;
					int64_t tau3 = tau2 * tau;
					int64_t tmp1 = As1 * tau3;
					int64_t tmp2 = Bs1 * tau2;
					int64_t tmp3 = Cs1 * tau;
					int64_t tmp4 = As2 * tau2;
					int64_t tmp5 = Bs2 * tau;
					int64_t p = ((tmp1 + tmp2 + tmp3) >> sh) + p0.position;
					int64_t s = ((tmp4 + tmp5) >> sh) + p0.speed;
					present.pmsm.pmsm.position_deseired = p;
					present.pmsm.pmsm.speed_force = s >> 1;
				}
			}
			bool first_point = true;

			void first_point_(point& _pt) {
				robo_infolog(RT("begin %u\t%d\t%d"), _pt.tm, (int)_pt.position, (int)_pt.speed);
				p1 = _pt;
				p0.tm = ClkCnt;
				#if pmsm_MOTOR_SPEED_FILTER_ENABLED == 1
				p0.speed = present.pmsm.pmsm.speed_filter.filtered;
				#else
				p0.speed = present.motor_enco.delta_acc;
				#endif
				p0.position = present.motor_enco.position;
				approxx_();
				if (first_point) {
					present.pmsm.pmsm.position_deseired = p0.position;
					first_point = false;
				}
				present.pmsm.pmsm.speed_force = 0;
			}
			void next_point_(point& _pt) {
				p0 = p1;
				p1 = _pt;
				approxx_();
				robo_infolog(RT("next %u\t%d\t%d"), _pt.tm, (int)_pt.position, (int)_pt.speed);
				first_point = false;

			}
			void freeze_(void) {
				first_point = false;
				robo_infolog(RT("freeze"));
			}
		public:
			int setPathPoint(uint32_t DAngle, uint16_t DVel, uint32_t ClkCntN) {
				if (path_.space()) {
					point& pt = path_.empty();
					pt.position = ils::angle_pp32_to_pp(DAngle);
					pt.speed = ils::spped_mg_to_pp(DVel);
					pt.tm = ClkCntN;
					path_.put();
					if (path_.count() == 1) {
						first_point_(pt);
					}
					return 0;
				}
				else {
					return -1;
				}
			}
			void clearPath(void) {
				path_.clear();
				freeze_();
			}
			void removeBufPnt(uint32_t ClkCnt) {
				if (path_.available()) {
					point& pt = path_.first();
					if ((int)(pt.tm - (uint32_t)ClkCnt) <= 0) {
						tractor_clock_ = pt.tm;
						path_.get();
						if (path_.available()) {
							next_point_(path_.first());
						}
						else {
							freeze_();
						}
					}
				}
			}
			#if ROBO_APP_MEXO_VAR_ENABLED == 1
			void var_reg(void) {
				::mexo::var::record::create(::mexo::var::types::const_int64, tau, RT("tr.tau"));
				::mexo::var::record::create(::mexo::var::types::const_uint32, p0.tm, RT("tr.p0.tm"));
				::mexo::var::record::create(types::var::const_signal, p0.speed, RT("tr.p0.sp"));
				::mexo::var::record::create(types::var::const_long_signal, p0.position, RT("tr.p0.po"));
				::mexo::var::record::create(::mexo::var::types::const_uint32, p1.tm, RT("tr.p1.tm"));
				::mexo::var::record::create(types::var::const_signal, p1.speed, RT("tr.p1.sp"));
				::mexo::var::record::create(types::var::const_long_signal, p1.position, RT("tr.p1.po"));
			}
			#endif
			tractor(controller& _controller) : posicioner(_controller) {}

		};
		#endif

		passive_task passive_task_;
		#if KALINA1_POSITION_SENCE_ENABLED == 1
		posicioner posicioner_;
		motion motion_;
		#endif
		#if KALINA1_TRACTOR_ENABLED == 1
		tractor tractor_;
		#endif


	protected:
		virtual void doTerminate(void) {
			action.pmsm.actuator.ps.dev.mode = pmsm::front::mode::idle;
			present.pmsm.actuator.ps.dev.mode = pmsm::front::mode::idle;
			#if KALINA1_BRAKE_ENABLED == 1
			action.br.ps.dev.mode = br::front::mode::idle;
			present.br.ps.dev.mode = br::front::mode::idle;
			present.br.command = ::mexo::brake::itf::command_t::set;
			#endif
			#if KALINA1_PAYLOAD_ENABLED
			payload_off();
			#endif
		}

		bool autoblock_ = true;

		//uint16_t reboot_delay_ms_ = 0;

		void mode_applay_(void) {
			switch (mode_) {
			case modes::off:
			stop();
			break;

			case modes::passive:
			switchto(&passive_task_);
			break;
			#if KALINA1_POSITION_SENCE_ENABLED == 1
			case modes::position:
			switchto(&posicioner_);
			break;
			case modes::motion:
			switchto(&motion_);
			break;
			#endif
			#if KALINA1_TRACTOR_ENABLED == 1
			case modes::tractor:
			switchto(&tractor_);
			break;
			#endif
			default:
			stop();
			break;
			}
		}
	public:
		#if KALINA1_TRACTOR_ENABLED == 1
		int setPathPoint(uint32_t DAngle, uint16_t DVel, uint32_t ClkCntN) {
			return tractor_.setPathPoint(DAngle, DVel, ClkCntN);
		}
		void clearPath(void) {
			tractor_.clearPath();
		}
		void removeBufPnt(uint32_t ClkCnt) {
			tractor_.removeBufPnt(ClkCnt);
		}
		#endif
		struct {
			types::long_signal_t min = 0;
			types::long_signal_t max = 0;
		} poslim;
		void set_mode(modes _mode) {
			if (present.pmsm.actuator.ps.dev.error == 0 && _mode != modes::unknown) {
				mode_ = _mode;
			}
			else {
				mode_ = modes::off;
			}
			if (mode_ == modes::off
				#if KALINA1_BRAKE_ENABLED == 1
				|| present.br.status != ::mexo::brake::itf::status_t::fixed
				#endif
				) {
				mode_applay_();
			}
		}

		modes mode(void) {
			return mode_;
		}
		statuses status(void) {
			return status_;
		}

		void brake_set(void) {
			switchto(nullptr);
		}
		void brake_release(void) {
			mode_applay_();
		}
		void terminate(void) {
			::mexo::controller::terminate();
			present.pmsm.actuator.ps.dev.error = (uint8_t)'T';
		}

		void protection_check_(void) {
			/*if (checkTirGuard() == 0) {
				ErrCtl |= ERRCTL_TIR;
				resetTirGuard();
				terminate();
			}
			if(checkWDT() == 0){
				ErrCtl |= ERRCTL_WDT;
				resetTirGuard();
				terminate();
			}*/
		}

		controller(void);
		virtual ~controller(void) {}
		#if KALINA1_POSITION_SENCE_ENABLED == 1
		bool position_mode(types::long_signal_t _angle, types::signal_t _speed_max, types::signal_t  _current, bool _autobrake) {
			if (present.pmsm.actuator.ps.dev.error == 0) {
				posicioner_.position_ = _angle;
				posicioner_.speed_ = _speed_max;
				posicioner_.current_ = _current;
				update_need = true;
				config.controller.autobrake = _autobrake;
				switchto(&posicioner_);
				return true;
			}
			else {
				return false;
			}
		}
		bool speed_mode(types::signal_t _speed, types::signal_t  _current) {
			if (present.pmsm.actuator.ps.dev.error == 0) {
				motion_.speed_ = _speed;
				motion_.current_ = _current;
				update_need = true;
				switchto(&motion_);
				return true;
			}
			else {
				return false;
			}
		}
		#endif

		bool passive_mode(void) {
			if (present.pmsm.actuator.ps.dev.error == 0) {
				switchto(&passive_task_);
				return true;
			}
			else {
				return false;
			}
		}

	} controller_;

	controller::controller(void)
		: passive_task_(*this)
		#if KALINA1_POSITION_SENCE_ENABLED == 1
		, posicioner_(*this)
		, motion_(*this)
		#endif
		#if KALINA1_TRACTOR_ENABLED == 1
		, tractor_(*this)
		#endif
	{
		#if KALINA1_TRACTOR_ENABLED == 1
		static ::mexo::machine::slot::simple interp(
			{ 3 }
			, [] {
				controller_.tractor_.interp();
			});
		#endif

		static ::mexo::machine::slot::simple run_(
			::mexo::machine::slot::kind::backend
			, [] {
				//controller_.set_mode(pet_drive::action.controller_mode);
				static ::robo::time_us_t last = 0;
				::robo::time_us_t now = ::robo::system::time_us();
				controller_.protection_check_();
				if (now - last > config.controller.pause.poll_us) {
					controller_.run();
					last = now;
				}
			}
		);
		static struct manual {
			types::signal_t current = 0;
			types::signal_t current_prev = 0;
			types::signal_t speed = 0;
			types::signal_t speed_prev = 0;
			types::long_signal_t position = 0;
			types::long_signal_t position_prev = 0;
			int mode = 0;
			int mode_prev = 0;
			bool enabled = true;
			void run(void) {
				if (enabled) {
					if (
						(mode_prev != mode)
						|| (current_prev != current)
						|| (speed_prev != speed)
						|| (position_prev != position)
						) {
						switch (mode) {
						case 1:
						controller_.passive_mode();
						break;
						#if PET_ROTOR_POSITION_SENCE_ENABLED == 1				
						case 3:
						controller_.speed_mode(speed, current);
						break;
						case 4:
						controller_.position_mode(position, speed, current, config.controller.autobrake);
						break;
						#endif
						default:
						controller_.stop();
						break;
						}
						mode_prev = mode;
						current_prev = current;
						speed_prev = speed;
						position_prev = position;
					}
				}
			}
		} manual_;
		static ::mexo::machine::slot::simple test_(
			::mexo::machine::slot::kind::frontend
			, [] {
				manual_.run();
			}
		);
	}

	//снимок
	struct snapshot : public ::robo::net::flow::snapshot {
		front::snapshot0_s raw = {};
		snapshot(void) {}
		virtual size_t size(void) { return front::snapshot0_s::mem_size; };
		virtual const uint8_t* data(void) { return raw.memo; };
		virtual void  update(void) {
			pmsm_.update_feedback();
//			feedback.controller.mode = controller_.mode();
			raw.encode(feedback);
		}
	} snapshot_;


	//обработчик команды snapshot для протокола FLOW
	::robo::net::flow::snapshot_proto snapshot_proto_(RT("SNAPSHOPT0"), ::robo::net::flow::snapshot_proto::kind_t::frontend, snapshot_);


	//добавляем обработчик команд snapshot в таблицу интерфейсов на соответствующий субадрес
	::robo::net::flow::rout_record flow_snapshot_rout_record_(
		ENV_NET_FLOW_PORT_PATH
		, RT("SNAPSHOPT0")
		, KALINA1_NET_FLOW_SNAPSHOT0_SUBA
		, KALINA1_NET_FLOW_SNAPSHOT0_SUBA_ANSV
	);


	//цель
	struct goal : public ::robo::net::flow::goal {
		kalina1_drive::front::goal0_s raw = {};
		goal(void) {}
		virtual void  applay(const uint8_t* _data, size_t _size) {
			raw.decode<types, controller>(controller_, _data, _size);
		}
	} goal_;

	//обработчик команды snapshot для протокола FLOW
	::robo::net::flow::goal_proto goal_proto_(RT("GOAL0"), ::robo::net::flow::snapshot_proto::kind_t::frontend, goal_, snapshot_proto_);
	//добавляем обработчик команд snapshot в таблицу интерфейсов на соответствующий субадрес
	::robo::net::flow::rout_record flow_echo_rout_record_(
		ENV_NET_FLOW_PORT_PATH
		, RT("GOAL0")
		, KALINA1_NET_FLOW_GOAL0_SUBA
		, KALINA1_NET_FLOW_SNAPSHOT0_SUBA_ANSV
	);
}