#ifndef INCLDE_CUBIC_SPLINE_INTERPOLATOR_
#define INCLDE_CUBIC_SPLINE_INTERPOLATOR_
#include "path_interpolator.hpp"

namespace interp{
/// Cubic Spline Interpolator
/// @brief parameters are kept internally.
/// @details Cubic spline is defined as.
///
/// ```
/// x(t) = a t^3 + b t^2 + c t + d
/// ```
class CubicSplineInterpolator : public PathInterpolator {
public:
  /// Constructor
  CubicSplineInterpolator();

  /// Destructor
  ~CubicSplineInterpolator();

  /// Generate tragectory path from Time,Position queue
  /// @param[in] Time,Position queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  /// @details
  /// Input is TimePosition Queue like this.
  ///
  /// ```
  /// (ts, xs), (t1, x1), (t2, x2), ..., (tf, xf)
  /// ```
  ///
  /// Interpolator set start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs=0, as=0, js=0),
  /// (t1, x1, v1,   a1,   j1),
  /// (t2, x2, v2,   a2,   j2),
  ///  ...,
  /// (tf, xf, vf=0, a_f=0, j_f=0)
  /// ```
  ///
  /// and interpolate (v1,a1,j1), (v1,a1,j1),.. automatically.
  virtual RetVal<double> generate_path( const TPQueue& tp_queue );

  /// Generate tragectory path from Time,Position(,Velocity) queue
  /// @param[in] Time,Position(,Velocity) queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  virtual RetVal<double> generate_path( const TPVQueue& tpv_queue );


  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param t input time
  /// - PATH_SUCCESS & TPV at the input time
  /// - PATH_NOT_GENERATED and TPV is time=-1.0, position=0.0, velocity=0.0
  virtual RetVal<TPV> pop(const double& t );
};

}

#endif // INCLDE_CUBIC_SPLINE_INTERPOLATOR_HPP_
