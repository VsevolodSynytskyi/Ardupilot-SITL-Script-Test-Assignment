#include "altctl/data_logger.hpp"

#include <cstdio>
#include <filesystem>

namespace altctl {

bool DataLogger::open(const std::string& path)
{
    path_ = path;
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }
    out_.open(path, std::ios::trunc);
    if (!out_) {
        return false;
    }
    out_ << "t_s,state,target_alt_m,setpoint_alt_m,alt_m,climb_sp_ms,climb_ms,thrust,"
            "p,i,d,i_frozen,dt_s\n";
    return true;
}

void DataLogger::write(const Row& r)
{
    if (!out_) {
        return;
    }
    char line[256];
    std::snprintf(line, sizeof(line),
                  "%.3f,%s,%.2f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%d,%.4f\n", r.t_s, r.state,
                  r.target_alt_m, r.ctl.alt_setpoint_m, r.alt_m, r.ctl.climb_setpoint_ms, r.climb_ms,
                  r.ctl.thrust, r.ctl.vel_terms.p, r.ctl.vel_terms.i, r.ctl.vel_terms.d,
                  r.ctl.integrator_frozen ? 1 : 0, r.dt_s);
    out_ << line;
    if (++rows_ % 50 == 0) {
        out_.flush();  // keep the log useful if the process dies
    }
}

}  // namespace altctl
