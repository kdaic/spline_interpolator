#ifndef INCLUDE_PATH_HPP_
#define INCLUDE_PATH_HPP_

#include "path_interpolator.hpp"
#include "cubic_spline_interpolator.hpp"


namespace path {

using namespace interp;

/// Path class
/// This class include the path interpolator object,
/// this is also the factory of some types interpolator object.
class Path {

public:
  /// Constructor
  Path();

  /// Destructor
  virtual ~Path();

private:
  PathInterpolator interp;
};

}

#endif // INCLUDE_PATH_HPP_
