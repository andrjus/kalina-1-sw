#include "core/robosd_system.hpp"
#include "core/robosd_log.hpp"
#include "core/robosd_string.hpp"
#include "kalina1_shared.hpp"

namespace kalina1_shared {
	robo::system::shared shared;
	kalina1_shared::content* content_ = nullptr;
	bool init(void) {
		ROBO_LBREAKN(shared.open(shared_name, sizeof(content)));
		content_ = (kalina1_shared::content*)shared.memo();
		content_->vrep = {};
		return true;
	}
	void cleanup(void) {
		shared.close();
	}
	void actuation(void) {}
	void sensing(void) {}
}

extern "C"{
	#define LUA_BUILD_AS_DLL
	#define LUA_LIB
	#include "lua.h"
	#include "lauxlib.h"

	
	int init(lua_State* L) {
		luaL_dostring(L, "print \"kalina1 init\"");
		kalina1_shared::init();
		return 1;
	}
	int cleanup(lua_State* L) {
		kalina1_shared::cleanup();
		//luaL_dostring(L, "print \"kalina1 cleanup\"");
		return 1;
	}

	int set_mode(lua_State* L) {
		int index = (int)lua_tonumber(L, 1);
		if ((index >= 0) && (index < kalina1_shared::drill::count)) {
			int mode = (int)lua_tonumber(L, 2);
			kalina1_shared::content_->drills[index].mode = mode;
			::robo::string s; s.format("print \"drill  %d set mode %d!\"", index, mode);
			luaL_dostring(L, s.c_str());
			return 1;
		} else  {
			::robo::string s; s.format("print \"eroor set mode %d!\"", index);
			luaL_dostring(L, s.c_str());
			return 0;
		}
	}

	int actuation(lua_State* L) {
		kalina1_shared::actuation();
		
		lua_newtable(L);
		lua_pushliteral(L, "status");
		lua_pushinteger(L, (lua_Integer)kalina1_shared::content_->side);
		lua_settable(L, -3);
		lua_pushliteral(L, "time");
		lua_pushnumber(L, kalina1_shared::content_->vrep.time);
		lua_settable(L, -3);

		lua_pushliteral(L, "drill1");
		lua_newtable(L);
		lua_pushliteral(L, "p");
		lua_pushnumber(L, kalina1_shared::content_->drill1.rotator);
		lua_settable(L, -3);
		lua_pushliteral(L, "s");
		lua_pushnumber(L, kalina1_shared::content_->drill1.supply);
		lua_settable(L, -3);
		lua_pushliteral(L, "w");
		lua_pushnumber(L, kalina1_shared::content_->drill1.power);
		lua_settable(L, -3);
		lua_settable(L, -3);

		lua_pushliteral(L, "drill2");
		lua_newtable(L);
		lua_pushliteral(L, "p");
		lua_pushnumber(L, kalina1_shared::content_->drill2.rotator);
		lua_settable(L, -3);
		lua_pushliteral(L, "s");
		lua_pushnumber(L, kalina1_shared::content_->drill2.supply);
		lua_settable(L, -3);
		lua_pushliteral(L, "w");
		lua_pushnumber(L, kalina1_shared::content_->drill2.power);
		lua_settable(L, -3);
		lua_settable(L, -3);


		if (kalina1_shared::content_->side == kalina1_shared::eside::vrep) {
			kalina1_shared::content_->side = kalina1_shared::eside::digitwin;
		}

		return 1;
	}
	int sensing(lua_State* L) {
		kalina1_shared::sensing();
		//luaL_dostring(L, "print \"kalina1 sensing\"");
		if (kalina1_shared::content_->side == kalina1_shared::eside::vrep) {
			kalina1_shared::content_->side = kalina1_shared::eside::digitwin;
		}
		return 1;
	}



	static luaL_Reg ls_lib[] = {
		{"init",init}
		, {"cleanup",cleanup}
		, {"actuation",actuation}
		, {"sensing",sensing}
		, {"set_mode",set_mode}
		, {nullptr, nullptr}
	};

	LUALIB_API int luaopen_kalina1_lua_client(lua_State* L) {
		luaL_newlib(L, ls_lib);
		luaL_dostring(L, "print \"kalina1 loaded!\"");
		return 1;
	}
}
