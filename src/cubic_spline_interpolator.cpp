#include "cubic_spline_interpolator.hpp"

using namespace interp;

CubicSplineInterpolator::CubicSplineInterpolator() {
}

CubicSplineInterpolator::~CubicSplineInterpolator() {
}

PathRetVal<double> CubicSplineInterpolator::generate_path(
                     const double& xs, const double& xf,
                     const double& vs, const double& vf,
                     const double& ts, const double& tf ) {
  if ( g_is_nearEq( ts, 0.0 ) && g_is_nearEq( tf, 0.0 ) ) {
    return PathRetVal<double>( PATH_NOT_DEF_100PER_PATH, -1.0 );
  }
  if ( ts < 0.0 || ts >= tf ) {
    return PathRetVal<double>( PATH_INVALID_INPUT_TIME, -1.0 );
  }

  set_TPVsf( ts, tf,  xs, xf,  ts, tf );

  return PathRetVal<double>( PATH_NOT_DEF_FUNCTION, tf - ts );
}

PathRetVal<double> CubicSplineInterpolator::genrate_path(
                     const TPQueue& tp_queue ) {
  tp_queue_ = tp_queue;
  std::size_t finish_index = tp_queue_.size() - 1;
  set_TPVsf( tp_queue_.get(0).time,     tp_queue_.get(finish_index).time,
             tp_queue_.get(0).position, tp_queue_.get(finish_index).position,
             0.0,                       0.0 );

  return PathRetVal<double>(PATH_SUCCESS, tf_-ts_);
}


PathRetVal<double> CubicSplineInterpolator::genrate_path(
                     const TPVQueue& tpv_queue ) {
  return PathRetVal<double>(PATH_NOT_DEF_FUNCTION, -1.0);
}
