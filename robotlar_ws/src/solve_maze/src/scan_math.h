#ifndef SOLVE_MAZE_SCAN_MATH_H
#define SOLVE_MAZE_SCAN_MATH_H

#include <cmath>
#include <cstddef>

namespace scan_math
{
inline bool validGeometry(double angle_min, double angle_increment, std::size_t count)
{
    return count > 0 && std::isfinite(angle_min) &&
           std::isfinite(angle_increment) && angle_increment != 0.0 &&
           std::isfinite(angle_min + angle_increment * static_cast<double>(count - 1));
}

inline bool angleIndex(double angle, double angle_min, double angle_increment,
                       std::size_t count, std::size_t &index)
{
    if (!validGeometry(angle_min, angle_increment, count) || !std::isfinite(angle))
        return false;

    const double turn = 2.0 * std::acos(-1.0);
    const double step = std::abs(angle_increment);
    double offset = std::remainder(angle, turn) - std::remainder(angle_min, turn);
    if (angle_increment < 0.0)
        offset = -offset;
    offset = std::fmod(offset + turn, turn);
    if (offset >= turn - step * 0.5)
        offset = 0.0;

    const double span = step * static_cast<double>(count - 1);
    if (offset > span + step * 0.5)
        return false;

    const double nearest = std::floor(offset / step + 0.5);
    if (nearest >= static_cast<double>(count))
        return false;
    index = static_cast<std::size_t>(nearest);
    return true;
}

inline double wallReading(double reading, double max_range)
{
    if (std::isnan(reading) || reading < 0.0)
        return 0.0;
    return reading > max_range ? max_range : reading;
}

inline bool mapPixel(double x, double y, double resolution, int width, int height,
                     int &pixel_x, int &pixel_y)
{
    const double px = width / 2 + x * resolution;
    const double py = height / 2 - y * resolution;
    if (!std::isfinite(px) || !std::isfinite(py) ||
        px < 0.0 || px >= width || py < 0.0 || py >= height)
        return false;
    pixel_x = static_cast<int>(px);
    pixel_y = static_cast<int>(py);
    return true;
}
}

#endif
