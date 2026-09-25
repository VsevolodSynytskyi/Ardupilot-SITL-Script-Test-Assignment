#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "altctl/config.hpp"
#include "altctl/data_logger.hpp"
#include "altctl/diagnostics.hpp"
#include "altctl/drone_interface.hpp"
#include "altctl/mission_runner.hpp"

namespace {

std::atomic<bool> g_stop{false};

// First Ctrl+C: stop the mission and land. Second Ctrl+C: quit immediately.
void on_signal(int)
{
    if (g_stop.exchange(true)) {
        std::_Exit(130);
    }
}

void usage(const char* argv0)
{
    std::cerr << "Usage: " << argv0
              << " [--config path] [--set key=value]... [--run mission|telemetry|open-loop]\n"
              << "  default config: config/mission.conf, default run: mission\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::string config_path = "config/mission.conf";
    std::string run = "mission";
    altctl::Config cfg;
    try {
        // --config first, so --set overrides are applied on top of the file
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--config" && i + 1 < argc) {
                config_path = argv[++i];
            }
        }
        cfg = altctl::Config::load(config_path);
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config") {
                ++i;
            } else if (arg == "--run" && i + 1 < argc) {
                run = argv[++i];
            } else if (arg == "--set" && i + 1 < argc) {
                const std::string kv = argv[++i];
                const auto eq = kv.find('=');
                if (eq == std::string::npos) {
                    throw std::runtime_error("--set expects key=value, got " + kv);
                }
                cfg.set(kv.substr(0, eq), kv.substr(eq + 1));
            } else {
                usage(argv[0]);
                return 2;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    // A closed stdout (e.g. the parent script exited) must not kill the process mid-flight.
    std::signal(SIGPIPE, SIG_IGN);

    altctl::DroneInterface drone(cfg.connection_url);
    if (!drone.connect(std::chrono::seconds(20))) {
        return 1;
    }
    if (run == "telemetry") {
        return altctl::run_telemetry_check(cfg, drone);
    }
    if (run == "open-loop") {
        return altctl::run_open_loop_takeoff(cfg, drone, g_stop);
    }
    if (run == "mission") {
        altctl::DataLogger logger;
        if (!logger.open(cfg.log_path)) {
            std::cerr << "cannot open log " << cfg.log_path << "\n";
            return 1;
        }
        std::cout << "[main] logging to " << cfg.log_path << "\n";
        altctl::MissionRunner mission(cfg, drone, logger, g_stop);
        return mission.run();
    }
    usage(argv[0]);
    return 2;
}
