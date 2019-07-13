#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"

using namespace interp;


/// Friend Test Class of CubicSplineInterpolator
class CubicSplineTest: public ::testing::Test
{
protected:
  CubicSplineTest() {}

  CubicSplineInterpolator cubic_spline_;
};
