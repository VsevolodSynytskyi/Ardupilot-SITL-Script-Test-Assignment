#pragma once

#include <fstream>
#include <string>

#include "altctl/altitude_controller.hpp"

namespace altctl {

// One CSV row per control tick, for scripts/plot.py.
class DataLogger {
public:
    struct Row {
        double t_s = 0.0;
        const char* state = "";
        double target_alt_m = 0.0;
        double alt_m = 0.0;
        double climb_ms = 0.0;
        double dt_s = 0.0;
        AltitudeController::Output ctl;
    };

    bool open(const std::string& path);  // creates parent directories
    void write(const Row& row);

private:
    std::ofstream out_;
    unsigned long rows_ = 0;
};

}  // namespace altctl
