//#include "windows.h"
#include "core/robosd_common.hpp"
#include "kalina1_model.hpp"
#include "net/platform/ip/robosd_udp_win.hpp"

#pragma comment(lib, "ws2_32.lib")

WSADATA wsaData_;

void kalina1_model::periphery_begin(void) {
	WSAStartup(MAKEWORD(2, 2), &wsaData_);
}
void kalina1_model::periphery_start(void) {
}
void kalina1_model::periphery_stop(void) {
}
void kalina1_model::periphery_finish(void) {
	WSACleanup();
}

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		break;
	case DLL_THREAD_ATTACH:
	break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

