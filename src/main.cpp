#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "altctl/config.hpp"
#include "altctl/data_logger.hpp"
#include "altctl/diagnostics.hpp"
#include "altctl/drone_interface.hpp"
#include "altctl/mission_runner.hpp"

namespace {

constexpr const char* kDefaultConfigPath = "config/mission.conf";
constexpr auto kConnectTimeout = std::chrono::seconds(20);
constexpr int kExitUsageError = 2;

std::atomic<bool> g_stop_requested{false};

// First Ctrl+C: stop the mission and land. Second Ctrl+C: quit immediately.
void handle_stop_signal(int)
{
    if (g_stop_requested.exchange(true)) {
        std::_Exit(altctl::MissionRunner::kExitInterrupted);
    }
}

void print_usage(const char* program)
{
    std::cerr << "Usage: " << program
              << " [--config path] [--set key=value]... [--run mission|telemetry|open-loop]\n"
              << "  default config: " << kDefaultConfigPath << ", default run: mission\n";
}

struct Options {
    std::string run_mode = "mission";
    altctl::Config config;
};

std::optional<Options> parse_options(int argc, char** argv)
{
    // --config first, so --set overrides are applied on top of the file
    std::string config_path = kDefaultConfigPath;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }
    Options options;
    options.config = altctl::Config::load(config_path);
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const bool has_value = i + 1 < argc;
        if (argument == "--config" && has_value) {
            ++i;
        } else if (argument == "--run" && has_value) {
            options.run_mode = argv[++i];
        } else if (argument == "--set" && has_value) {
            const std::string assignment = argv[++i];
            const auto separator = assignment.find('=');
            if (separator == std::string::npos) {
                throw std::runtime_error("--set expects key=value, got " + assignment);
            }
            options.config.set(assignment.substr(0, separator), assignment.substr(separator + 1));
        } else {
            return std::nullopt;
        }
    }
    return options;
}

int run_mission(const altctl::Config& config, altctl::DroneInterface& drone)
{
    altctl::DataLogger logger;
    if (!logger.open(config.log_path)) {
        std::cerr << "cannot open log " << config.log_path << "\n";
        return altctl::MissionRunner::kExitError;
    }
    std::cout << "[main] logging to " << config.log_path << "\n";
    altctl::MissionRunner mission(config, drone, logger, g_stop_requested);
    return mission.run();
}

}  // namespace

int main(int argc, char** argv)
{
    std::optional<Options> options;
    try {
        options = parse_options(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return kExitUsageError;
    }
    if (!options || (options->run_mode != "mission" && options->run_mode != "telemetry" &&
                     options->run_mode != "open-loop")) {
        print_usage(argv[0]);
        return kExitUsageError;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    // A closed stdout (e.g. the parent script exited) must not kill the process mid-flight.
    std::signal(SIGPIPE, SIG_IGN);

    altctl::DroneInterface drone(options->config.connection_url);
    if (!drone.connect(kConnectTimeout)) {
        return altctl::MissionRunner::kExitError;
    }
    if (options->run_mode == "telemetry") {
        return altctl::run_telemetry_check(drone);
    }
    if (options->run_mode == "open-loop") {
        return altctl::run_open_loop_takeoff(options->config, drone, g_stop_requested);
    }
    return run_mission(options->config, drone);
}
