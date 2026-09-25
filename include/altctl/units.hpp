#pragma once

namespace altctl {

inline constexpr double kPi = 3.14159265358979323846;

constexpr double degrees_to_radians(double degrees)
{
    return degrees * kPi / 180.0;
}

constexpr double radians_to_degrees(double radians)
{
    return radians * 180.0 / kPi;
}

}  // namespace altctl
