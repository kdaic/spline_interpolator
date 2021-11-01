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

  /// Copy Constructor
  /// @param[in] src source to copy SplineInterpolator
  SplineInterpolator( const SplineInterpolator& src );

  /// Copy Constructor - value copy
  /// @param[in] is_path_generated   flag if the spline-path is generated
  /// @param[in] is_v_limitation    flag if velocity limit (v_limit) is defined or not
  /// @param[in] v_limit            limit velocity [m/s, rad/s, ...etc.]
  /// @param[in] target_tpva_queue  target TPVQueue
  SplineInterpolator( const bool&      is_path_generated,
                      const bool&      is_v_limit,
                      const double&    v_limit,
                      const TPVAQueue& target_tpva_queue );

  /// Get total interval time
  /// @return total interval time of spline-path
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double total_dT() const;

  /// Get limit of velocity( if exists )
  /// @return limit velocity
  /// @exception
  /// - NoVelocityLimit : velocity limit is not defined
  const double v_limit() const;

  /// Get start time
  /// @return start time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double start_time() const;

  /// Get finish time
  /// @return finish time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double finish_time() const;

  /// Get the trajectory index of the input time
  /// @param[in] t            input time
  /// @param[in] output_index output index matched the input time
  /// @return
  /// - SPLINE_INVALID_INPUT_TIME : fail. The input time is out of range of target_tpva_queue.
  /// - SPLINE_SUCCESS            : success.
  /// @exception
  /// - QueueSizeEmpty : failed to pop a point from trajectory
  ///                    because the size of target_tpva_queue is zero.
  const RetCode index_of_time( const double& t,
                               std::size_t&  output_index ) const;

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

  /// Generate a spline-path from start and finish Position(, Velocity, Acceleration) queue
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
  virtual const TimePVA pop( const double& t )const = 0;

  /// clear target TPVAQueue (target_tpva_queue_)
  ///       & path parameter queue (depend on each interpolator class)
  virtual RetCode clear();

  /// Return the size of target TPVAQueue (target_tpva_queue_)
  /// @return targt_tpva_queue_
  virtual const std::size_t target_tpva_queue_size() const;

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
