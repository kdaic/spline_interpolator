#ifndef INCLUDE_SPLINE_HPP_
#define INCLUDE_SPLINE_HPP_

#include "spline_interpolator.hpp"
#include "cubic_spline_interpolator.hpp"


using namespace interp;

namespace spline {

/// Path class
/// This class include the path interpolator object,
/// this is also the factory of some types interpolator object.
class Spline {

public:
  /// Constructor
  Spline();

  /// Destructor
  virtual ~Spline();

private:
  SplineInterpolator* interp;
};

}

#endif // INCLUDE_PATH_HPP_
