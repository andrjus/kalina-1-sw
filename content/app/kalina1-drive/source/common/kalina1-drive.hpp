#ifndef kalina1_drive_hpp
#define kalina1_drive_hpp

#include "kalina1-drive.proto.hpp"



//подключаем и конфигурируем (см. kalina1-drive.config.hpp) шаблон привода 
#define PMSM_TEMPLATE_NAME KALINA1_DRIVE_PMSM_NAME
#define PMSM_ACTUATOR_SUB_NAME KALINA1_DRIVE_ACTUATOR_NAME
#define PMSM_PS_CROSS_NAME KALINA1_DRIVE_PS_CROSS_NAME
#define PMSM_PS_LAT_NAME KALINA1_DRIVE_PS_LAT_NAME
#include "mexo/pmsm.templ.inc.hpp"

#if KALINA1_BRAKE_ENABLED == 1
//подключаем и конфигурируем (см. kalina1-drive.config.hpp) шаблон привода 
#define BRAKE_TEMPLATE_NAME KALINA1_BRAKE_NAME
#define BRAKE_PS_TEMPLATE_NAME KALINA1_BRAKE_PS_NAME
#include "mexo/brake.templ.inc.hpp"
#endif

#include "mexo/mexo.hpp"
#include "mexo/mexo.env.common.hpp"
#include "mexo/math.hpp"
#include "mexo/ps.hpp"
#include "mexo/enco.hpp"
#include "net/robosd_flow.hpp"
#include "net/robosd_flow_id.h"
#include "mexo/pinout.hpp"

namespace kalina1_drive{
	typedef mexo_drive_types_t types;
	// аппаратура
	namespace periphery {
		//окружение - часы и прочее
		namespace env {
			void begin(void);
			void start(void);
		}
		//собственно, can
		namespace can {
			void begin(uint32_t  _bitrate, uint8_t _addr);
			void start(void);
		}

		//драйвер ШИМ для мотора см "mexo/ps.hpp
		struct  pwm {
			static void  begin(void);
			static void  start(void);
			typedef ::mexo::ps::abc_inverter<types> inverter;
			static void boot_complete(const inverter::duty_t& _duty);
			static void shutdown_begin(void);
			static void do_run(const inverter::duty_t& _duty);

			static void boot_begin(void);
			static bool do_boot(void);

			static bool do_shutdown(void);
			static void shutdown_complete(void);
		};

		#if KALINA1_BRAKE_ENABLED == 1
		//драйвер ШИМ для тормоза см "mexo/ps.hpp
		struct  br_pwm {
			static void  begin(void);
			typedef ::mexo::ps::dc_inverter<types> inverter;
			static void boot_complete(const inverter::duty_t& _duty);
			static void shutdown_begin(void);
			static void do_run(const inverter::duty_t& _duty);

			static void boot_begin(void);
			static bool do_boot(void);

			static bool do_shutdown(void);
			static void shutdown_complete(void);
		};
		#endif
		//драйвер АЦП см "mexo/adc.hpp
		struct adc {
			enum { channel_count = KALINA1_ADC_CH_COUNT };
			static void  begin(void);
			static uint32_t raw[channel_count];
			static void query(void);
			static void sample(void);
		};

		namespace enco {
			namespace rotor {
				#if KALINA1_MOTOR_INC_ENABLED == 1
				//драйвер  энкодера положения ротора на инкрементном датчике см "mexo/enco.hpp
				class increment {
				public:
					typedef uint16_t	unative_t;
					typedef int16_t		native_t;
					static void			begin(void);
					static uint32_t		encode(void);
					static void			query(void);
					static bool 		error(void);
				};
				#endif

				#if KALINA1_ROTOR_HALL_ENABLED == 1
				//драйвер  энкодера положения роторана холах см "mexo/enco.hpp
				class hall {
				public:
					typedef uint32_t	unative_t;
					typedef int32_t		native_t;
					static void			begin(void);
					static uint32_t		encode(void);
					static void			query(void);
					static bool 		error(void);
				};
				#endif

				#if KALINA1_MOTOR_ENCO_ABS_ENABLED == 1
				//драйвер  энкодера на холах см "mexo/enco.hpp
				class abs {
				public:
					typedef uint16_t	unative_t;
					typedef int16_t		native_t;					
					static uint16_t		encode(void);
					static void			query(void) {};
					static bool 		error(void) { return false;  };
				};
				#endif
			}
			namespace actuator {
				#if KALINA1_ACTUATOR_ENCO_ABS_ENABLED == 1
				//драйвер  энкодера на холах см "mexo/enco.hpp
				class abs {
				public:
					typedef uint32_t	unative_t;
					typedef int32_t		native_t;
					static void			begin(void);
					static uint32_t		encode(void);
					static void			query(void);
					static bool 		error(void);
				};
				#endif
			}
		}
		//дополнительная контрольная точка
		class tp2 {
		protected:
			static void on(void);
			static void off(void);
		};
	}

	//силовой преобразователь для мотора
	typedef ::mexo::handler_block_t < ::mexo::ps::pwm <periphery::pwm, const  ::mexo::cs_t<types>& >, ::mexo::backend_subsystem, const  ::mexo::cs_t<types>&  >
		abc_power_supply;

	#if KALINA1_BRAKE_ENABLED == 1
	//силовой преобразователь для тормоза
	typedef ::mexo::handler_block_t < ::mexo::ps::pwm <periphery::br_pwm>, ::mexo::backend_subsystem  > br_power_supply;
	#endif

	//датчик тока мотора
	typedef ::mexo::handler_block_t < ::mexo::adc::current_abc_sence <types, periphery::adc>, ::mexo::realtime_subsystem, const  ::mexo::cs_t<types>&  > current_abc_sensor;
	//датчик тока тормоза
	typedef ::mexo::handler_block_t < ::mexo::adc::current_sence <types, periphery::adc>, ::mexo::backend_subsystem > br_current_sensor;


	//датчики положения
	#if KALINA1_ROTOR_HALL_ENABLED == 1
	typedef ::mexo::handler_task_t <  ::mexo::enco::increment32_t<types, BiSS, KALINA1_MOTOR_ENCO_DRIVER_BITS, KALINA1_MOTOR_ENCO_ACTUAL_BITS>, ::mexo::periodic_subsystem > motor_enco;
	#endif

	#if KALINA1_MOTOR_INC_ENABLED == 1
	typedef ::mexo::handler_task_t <  ::mexo::enco::abs16_t<types, periphery::enco::rotor::increment, KALINA1_MOTOR_ENCO_ROTOR_INC_BITS, KALINA1_MOTOR_ENCO_ROTOR_INC_BITS>,  ::mexo::periodic_task > motor_enco;
	#endif

	#if KALINA1_MOTOR_ENCO_ABS_ENABLED == 1
	typedef ::mexo::handler_task_t <  ::mexo::enco::abs16_t<types, periphery::enco::rotor::abs, KALINA1_MOTOR_ENCO_ABS_BITS, KALINA1_MOTOR_ENCO_ABS_BITS>, ::mexo::periodic_task > motor_enco;
	#endif

	#if KALINA1_ACTUATOR_ENCO_ABS_ENABLED == 1
	typedef ::mexo::handler_task_t <  ::mexo::enco::abs32_t<types, periphery::enco::actuator::abs, KALINA1_ACTUATOR_ENCO_ABS_BITS, KALINA1_ACTUATOR_ENCO_ABS_BITS>, ::mexo::periodic_task > actuator_enco;
	#endif

	//ротатор
	typedef	::mexo::function_block_t < ::mexo::enco::rotator_t<types, uint16_t>, ::mexo::realtime_subsystem > rotator;
	
	//контрольная точка
	extern ::mexo::led<periphery::tp2> tp2;

}

#endif