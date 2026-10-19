#include "CommBridge.h"

#include <MSFS/MSFS_CommBus.h>

#include "Json.h"
#include "Log.h"
#include "Platform.h"

namespace br {

void CommBridge::open() {
    open_ = fsCommBusRegister(kCommandEvent, &CommBridge::onCommandThunk, this);
    if (open_) {
        log::info("CommBus: listening on %s", kCommandEvent);
    } else {
        log::error("CommBus: fsCommBusRegister(%s) failed; the panel will not work", kCommandEvent);
    }
}

void CommBridge::close() {
    if (open_) fsCommBusUnregister(kCommandEvent, &CommBridge::onCommandThunk);
    open_ = false;
}

void CommBridge::onCommandThunk(const char* buf, unsigned int size, void* ctx) {
    static_cast<CommBridge*>(ctx)->onCommand(buf, size);
}

void CommBridge::send(const char* event, const std::string& json) {
    // Include the terminator: JS receives the buffer as a C string.
    if (!fsCommBusCall(event, json.c_str(), static_cast<unsigned int>(json.size() + 1), FsCommBusBroadcast_JS)) {
        log::warn("CommBus: fsCommBusCall(%s) failed", event);
    }
}

void CommBridge::reply(int id, const std::string& op, bool ok, const std::string& error) {
    out_.clear();
    JsonWriter w(out_);
    w.beginObject().kv("id", id).kv("op", op).kv("ok", ok).kv("error", error).endObject();
    send(kReplyEvent, out_);
}

void CommBridge::onCommand(const char* buf, unsigned int size) {
    size_t len = 0;
    while (buf && len < size && buf[len] != '\0') ++len;
    JsonValue cmd;
    std::string err;
    if (!parseJson(buf, len, cmd, &err) || !cmd.isObject()) {
        log::warn("CommBus: ignoring malformed command (%s)", err.c_str());
        return;
    }
    const std::string op = cmd.getString("op", "");
    const int id = static_cast<int>(cmd.getNumber("id", 0.0));
    if (op != "status") log::debug("command %s (id %d)", op.c_str(), id);

    if (op == "status") {
        engine_.statusJson(out_);
        send(kStatusEvent, out_);
        return;
    }
    if (op == "clips") {
        const double offset = cmd.getNumber("offset", 0.0);
        engine_.clipsJson(offset > 0 ? static_cast<size_t>(offset) : 0, kClipsPerPage, out_);
        send(kClipsEvent, out_);
        return;
    }

    bool ok = true;
    if (op == "saveLast") {
        ok = engine_.saveLast(utcNowMs(), err);
    } else if (op == "delete") {
        ok = engine_.deleteClip(cmd.getString("file", ""), err);
    } else if (op == "play") {
        ok = engine_.play(cmd.getString("file", ""), err);
    } else if (op == "pause") {
        ok = engine_.pausePlayback(err);
    } else if (op == "resume") {
        ok = engine_.resumePlayback(err);
    } else if (op == "seek") {
        ok = engine_.seek(cmd.getNumber("t", 0.0), err);
    } else if (op == "speed") {
        ok = engine_.setSpeed(cmd.getNumber("x", 1.0), err);
    } else if (op == "stop") {
        ok = engine_.stopPlayback(err);
    } else if (op == "setupRequest") {
        ok = engine_.setupRequest(err);
    } else if (op == "setupStart") {
        ok = engine_.setupStart(err);
    } else if (op == "setupAnswer") {
        ok = engine_.setupAnswer(cmd.getString("a", ""), err);
    } else if (op == "setupLater") {
        ok = engine_.setupLater(err);
    } else if (op == "setupNever") {
        ok = engine_.setupNever(err);
    } else if (op == "setupReset") {
        ok = engine_.setupReset(err);
    } else if (op == "setDebug") {
        log::g_debug = cmd.getBool("on", false);
        log::info("debug logging %s", log::g_debug ? "on" : "off");
    } else {
        ok = false;
        err = "unknown command: " + op;
    }
    if (!ok) log::warn("command %s failed: %s", op.c_str(), err.c_str());
    reply(id, op, ok, err);
}

}  // namespace br
