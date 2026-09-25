#pragma once

#include <cstddef>
#include <fstream>
#include <string>

#include "altctl/altitude_controller.hpp"

namespace altctl {

// One CSV row per control tick, for scripts/plot.py.
class DataLogger {
public:
    struct Row {
        double time_s = 0.0;
        const char* state = "";
        double target_alt_m = 0.0;
        double alt_m = 0.0;
        double climb_mps = 0.0;
        double loop_period_s = 0.0;
        AltitudeController::Output controller;
    };

    [[nodiscard]] bool open(const std::string& path);  // creates parent directories
    void write(const Row& row);

private:
    std::ofstream file_;
    std::size_t rows_written_ = 0;
};

}  // namespace altctl
