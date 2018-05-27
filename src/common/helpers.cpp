#include "helpers.h"

#include <string>
#include <math.h>


string helpers::hasData(const string s) {
    const auto found_null = s.find("null");
    const auto b1 = s.find_first_of("[");
    const auto b2 = s.rfind("}]");

    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";
}


double helpers::normalizeAngle(double angle) {
    const int times = angle / M_PI;

    return angle - M_PI * (times >= 0 ? ceil(times) : floor(times));
}
