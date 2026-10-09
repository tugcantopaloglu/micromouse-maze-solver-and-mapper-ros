#include "../src/scan_math.h"
#include <cassert>
#include <limits>

int main()
{
    const double pi = std::acos(-1.0);
    std::size_t index = 999;
    assert(scan_math::angleIndex(-pi / 2, 0.0, pi / 180, 360, index) && index == 270);
    assert(scan_math::angleIndex(0.0, 0.0, pi / 180, 360, index) && index == 0);
    assert(scan_math::angleIndex(-pi / 2, -pi, pi / 180, 360, index) && index == 90);
    assert(scan_math::angleIndex(-pi / 2, 0.0, -pi / 180, 360, index) && index == 90);
    assert(scan_math::angleIndex(-pi / 2, 0.0f, static_cast<float>(pi / 180), 360, index) && index == 270);
    assert(scan_math::angleIndex(0.0, -static_cast<float>(pi), static_cast<float>(pi / 180), 360, index) && index == 180);
    assert(!scan_math::angleIndex(-pi / 2, 0.0, pi / 180, 181, index));
    assert(scan_math::angleIndex(pi / 4, -pi / 2, pi / 180, 136, index) && index == 135);
    assert(!scan_math::angleIndex(0.0, 0.0, 0.0, 360, index));
    assert(!scan_math::angleIndex(0.0, 0.0, pi / 180, 0, index));
    assert(!scan_math::angleIndex(0.0, std::numeric_limits<double>::quiet_NaN(), pi / 180, 360, index));
    assert(scan_math::wallReading(std::numeric_limits<double>::quiet_NaN(), 8.0) == 0.0);
    assert(scan_math::wallReading(-std::numeric_limits<double>::infinity(), 8.0) == 0.0);
    assert(scan_math::wallReading(std::numeric_limits<double>::infinity(), 8.0) == 8.0);
    assert(scan_math::wallReading(-1.0, 8.0) == 0.0);
    assert(scan_math::wallReading(0.35, 8.0) == 0.35);
    int x = -1, y = -1;
    assert(scan_math::mapPixel(1.0, -1.0, 20.0, 600, 600, x, y) && x == 320 && y == 320);
    assert(!scan_math::mapPixel(std::numeric_limits<double>::quiet_NaN(), 0.0, 20.0, 600, 600, x, y));
    assert(!scan_math::mapPixel(std::numeric_limits<double>::max(), 0.0, 20.0, 600, 600, x, y));
    assert(!scan_math::mapPixel(-15.01, 0.0, 20.0, 600, 600, x, y));
    return 0;
}
