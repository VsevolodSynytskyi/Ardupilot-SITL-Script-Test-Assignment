#include "altctl/data_logger.hpp"

#include <cstdio>
#include <filesystem>

namespace altctl {

namespace {

constexpr std::size_t kFlushEveryRows = 50;  // keep the log useful if the process dies

}  // namespace

bool DataLogger::open(const std::string& path)
{
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ignored;
        std::filesystem::create_directories(parent, ignored);
    }
    file_.open(path, std::ios::trunc);
    if (!file_) {
        return false;
    }
    file_ << "time_s,state,target_alt_m,setpoint_alt_m,alt_m,climb_setpoint_mps,climb_mps,thrust,"
             "p,i,d,integrator_frozen,loop_period_s\n";
    return true;
}

void DataLogger::write(const Row& row)
{
    if (!file_) {
        return;
    }
    const auto& controller = row.controller;
    char line[256];
    std::snprintf(line, sizeof(line),
                  "%.3f,%s,%.2f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%d,%.4f\n", row.time_s,
                  row.state, row.target_alt_m, controller.setpoint_alt_m, row.alt_m,
                  controller.climb_setpoint_mps, row.climb_mps, controller.thrust,
                  controller.velocity_pid_terms.p, controller.velocity_pid_terms.i,
                  controller.velocity_pid_terms.d, controller.integrator_frozen ? 1 : 0,
                  row.loop_period_s);
    file_ << line;
    if (++rows_written_ % kFlushEveryRows == 0) {
        file_.flush();
    }
}

}  // namespace altctl
