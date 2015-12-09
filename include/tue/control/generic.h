#ifndef TUE_CONTROL_GENERIC_H_
#define TUE_CONTROL_GENERIC_H_

#include <limits>
#include <cmath>

namespace tue
{
namespace control
{

static double INVALID_DOUBLE = std::numeric_limits<double>::quiet_NaN();

inline bool is_set(double v) { return !std::isnan(v); }

} // end namespace tue

} // end namespace control

#endif
