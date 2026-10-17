// BetterReplay standalone WASM module entry points.
#include <MSFS/MSFS.h>

#include "CommBridge.h"
#include "Engine.h"
#include "FileStore.h"
#include "Log.h"
#include "SimBridge.h"

namespace {

constexpr const char* kVersion = "0.1.0";

void engineLog(void*, int level, const char* msg) {
    if (level >= 2) {
        br::log::error("%s", msg);
    } else if (level == 1) {
        br::log::warn("%s", msg);
    } else {
        br::log::info("%s", msg);
    }
}

struct App {
    br::FileStore files;
    br::Engine engine{files};
    br::SimBridge sim{engine};
    br::CommBridge comm{engine};

    void init() {
        br::log::info("BetterReplay %s starting", kVersion);
        engine.setLogger(&engineLog, nullptr);
        engine.setSimOutput(&sim);
        engine.setControlSim(&sim);
        if (files.probe()) {
            // A copy of the log in the work folder, so it can be read after a session without
            // the DevMode console. The previous session's log is kept next to it.
            br::log::openFile(files.dir() + "betterreplay.log", files.dir() + "betterreplay.prev.log");
            br::log::info("BetterReplay %s, debug logging %s", kVersion, br::log::g_debug ? "on" : "off");
        }
        engine.init();
        sim.open();
        comm.open();
    }

    void shutdown() {
        comm.close();
        sim.close();
        br::log::info("BetterReplay stopped");
        br::log::closeFile();
    }
};

App* g_app = nullptr;

}  // namespace

extern "C" MSFS_CALLBACK void module_init(void) {
    g_app = new App();
    g_app->init();
}

extern "C" MSFS_CALLBACK void module_post_sim_systems_update(float dTime) {
    if (g_app) g_app->sim.update(dTime);
}

extern "C" MSFS_CALLBACK void module_deinit(void) {
    if (!g_app) return;
    g_app->shutdown();
    delete g_app;
    g_app = nullptr;
}
