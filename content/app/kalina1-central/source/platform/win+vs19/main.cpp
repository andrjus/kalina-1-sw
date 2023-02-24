#include "core/robosd_app.hpp"
#include "core/robosd_system.hpp"
#include "core/robosd_ini.hpp"
#include <thread>
#include <iostream>
#include <chrono>
#include <thread>
#include "im/edev/edev.hpp"
#if ROBO_UNICODE_ENABLED == 1
int wmain(int _argc, wchar_t* _argv[]){
	const wchar_t* ini = 0;
#else
int main(int _argc, char* _argv[]){
	const char* ini = 0;
#endif

	bool enulator_enabled = false;
	typedef std::chrono::high_resolution_clock Time;
	typedef std::chrono::duration<double> fsec;

	auto t0 = Time::now();

	if (_argc > 1) {
		ini = _argv[1];
	}
	else {
		ini = RT("kalina1-central.ini");
	}

	ROBO_JAMPN(robo::system::consol::begin([&](robo::system::consol::event /**/) {robo::app::machine::stop(); }), crash);
	robo_infolog("kalina1's central begin");
	ROBO_JAMPN(robo::app::machine::begin(ini), crash);
	ROBO_JAMPN(robo::app::machine::start(), crash);
	ROBO_JAMPN(robo::ini::load(RT("SETTINGS"), RT("EMULATOR_ENABLED"), enulator_enabled), crash);

	if (enulator_enabled) {
		if (_argc > 2) {
			ini = _argv[2];
		}
		else {
			ini = RT("robot-im.ini");
		}

		ROBO_JAMPN(robo::edev::agent::begin(ini), crash);
	}
	//ROBO_JAMPN(robo::system::ini::begin(ini), crash);
	{
		std::thread th([] {
			while (!robo::app::machine::terminated()) {
				robo::app::machine::frontend_loop();
			}
		});
		std::thread th2(
			[&]() {
				while (!robo::app::machine::terminated()) {
					if (enulator_enabled) {
						auto t1 = Time::now();
						fsec fs = t1 - t0;
						double sec = fs.count();
						robo::edev::agent::backgrounf_run(sec);
					}
					else {
						using namespace std::chrono_literals;
						std::this_thread::sleep_for(2000ms);
					}
				}
			}
		);
		std::thread th3(
			[&]() {
				while (!robo::app::machine::terminated()) {
					if (enulator_enabled) {
						auto t1 = Time::now();
						fsec fs = t1 - t0;
						double sec = fs.count();
						robo::edev::agent::run(sec);
					}
					else {
						using namespace std::chrono_literals;
						std::this_thread::sleep_for(2000ms);
					}
				}
			}
		);
		while (!robo::app::machine::terminated()) {
			robo::app::machine::backend_loop();
		}
		th.join();
		th2.join();
		th3.join();
	}
crash:

	robo::app::machine::finish();
	if (enulator_enabled) {
		robo::edev::agent::finish();
	}
	robo::system::consol::finish();
	robo_infolog("kalina1's central was finished");
	char c;
	std::cin >> c;
	return 0;
}