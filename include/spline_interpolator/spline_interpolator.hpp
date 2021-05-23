#ifndef INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
#define INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_

#include "spline_data.hpp"

namespace interp {

/// Base class of spline-path interpolator
class SplineInterpolator
{
public:
  /// Constructor
  SplineInterpolator();

  /// Destructor
  virtual ~SplineInterpolator();

  /// Get total interval time
  /// @return total interval time of spline-path
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double total_dT();

  /// Get limit of velocity( if exists )
  /// @return limit velocity
  /// @exception
  /// - NoVelocityLimit : velocity limit is not defined
  const double v_limit();

  /// Get finish v_limit( );

  /// Get finish time
  /// @return finish time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double finish_time();

  /// Genrate a spline-path from initial-finish point
  /// @paran[in] ts start time (default: 0.0)
  /// @paran[in] tf finish time (default: 0.0)
  /// @param[in] xs start position
  /// @param[in] xf finish position
  /// @param[in] vs start velocity (default: 0.0)
  /// @param[in] vf finish velocity (default: 0.0)
  /// @param[in] as start acceleration (default: 0.0)
  /// @param[in] af finish acceleration (default: 0.0)
  /// @brief calcurate tragectory parameter
  /// @return
  /// - SPLINE_SUCCESS
  /// @details
  /// If you don't give interval time(dT=0.0),
  /// Minmum interval time dT are internally calculated automatically. \n
  /// This means 100% mimum-time spline-path in the limitation.
  virtual RetCode generate_path( const double& ts, const double& tf,
                                 const double& xs, const double& xf,
                                 const double& vs=0.0, const double& vf=0.0,
                                 const double& as=0.0, const double& af=0.0 );

  /// Generate a spline-path from Time, Position queue
  /// @param[in] target_tp_queue target Time,Position queue
  /// @param[in] vs              start velocity (default: 0.0)
  /// @param[in] vf              finish velocity (default: 0.0)
  /// @return
  /// - SPLINE_SUCCESS
  /// @details
  /// Input is TimePosition Queue like this.
  ///
  /// ```
  /// (ts, xs (, vs, as)), (t1, x1), (t2, x2), ..., (tf, xf(, vf, af))
  /// ```
  ///
  /// Interpolator sets start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs,   as=0, js=0),
  /// (t1, x1, v1=?, a1=?, j1=?),
  /// (t2, x2, v2=?, a2=?, j2=?),
  ///  ...,
  /// (tf, xf, vf,   af=0, jf=0)
  /// ```
  ///
  /// and interpolates intermediate (v1,a1,j1), (v2,a2,j2),.. automatically.
  virtual RetCode generate_path( const TPQueue& target_tp_queue,
                                 const double vs=0.0, const double vf=0.0,
                                 const double as=0.0, const double af=0.0)=0;

  /// Generate a spline-path from Time, Position(, Velocity) queue
  /// @param[in] target_tpva_queue target Time, Position(, Velocity, Acceleration) queue
  /// @return
  /// - SPLINE_SUCCESS
  virtual RetCode generate_path( const TPVAQueue& target_tpva_queue )=0;

  /// Generate a spline-path from Position(, Velocity, Acceleration) queue
  /// @param[in] xs start position
  /// @param[in] xf finish position
  /// @param[in] vs start velocity (default: 0.0)
  /// @param[in] vf finish velocity (default: 0.0)
  /// @param[in] as start acceleration (default: 0.0)
  /// @param[in] af finish acceleration (default: 0.0)
  /// @return
  /// - SPLINE_SUCCESS and total interval time (tf - ts)
  /// you only give position & velocity & acceleration and don't give time. \n
  ///
  /// ```
  /// (xs, vs, as), (xf, vf, af)
  /// ```
  ///
  /// Minmum interval time -> dT(ts=0, tf) are internally calculated automatically. \n
  /// This means 100% mimum-time spline-path in the limitation.
  virtual RetCode generate_path_from_pva( const double& xs,     const double& xf,
                                          const double& vs,     const double& vf,
                                          const double& as=0.0, const double& af=0.0 )=0;

  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param[in] t input time
  /// @return output TPV at the input time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  /// - TimeOutOfRange : time is not within the range of generated spline-path
  virtual const TimePVA pop( const double& t )=0;

protected:
  /// flag if the spline-path is generated. (default: false)
  bool is_path_generated_;

  /// flag if velocity limit (v_limit) is defined or not (default: false)
  bool is_v_limit_;

  /// limit velocity [m/s, rad/s, ...etc.]
  /// (if defined)
  double v_limit_;

  /// target TPVQueue
  TPVAQueue target_tpva_queue_;
}; // End of class SplineInterpolator

} // End of namespace interp
#endif // INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
