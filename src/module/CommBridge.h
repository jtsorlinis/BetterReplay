// CommBus adapter. The toolbar panel is a stateless remote: it sends JSON commands on
// "BetterReplay.Cmd" and receives "BetterReplay.Status", "BetterReplay.Clips" and
// "BetterReplay.Reply" messages.
#pragma once

#include <string>

#include "Engine.h"

namespace br {

class CommBridge {
public:
    explicit CommBridge(Engine& engine) : engine_(engine) {}

    void open();
    void close();

    static constexpr const char* kCommandEvent = "BetterReplay.Cmd";
    static constexpr const char* kStatusEvent = "BetterReplay.Status";
    static constexpr const char* kClipsEvent = "BetterReplay.Clips";
    static constexpr const char* kReplyEvent = "BetterReplay.Reply";
    static constexpr size_t kClipsPerPage = 20;

private:
    static void onCommandThunk(const char* buf, unsigned int size, void* ctx);
    void onCommand(const char* buf, unsigned int size);
    void send(const char* event, const std::string& json);
    void reply(int id, const std::string& op, bool ok, const std::string& error);

    Engine& engine_;
    bool open_ = false;
    std::string out_;  // reused for every outgoing message
};

}  // namespace br
