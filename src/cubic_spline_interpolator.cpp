#include "cubic_spline_interpolator.hpp"

using namespace interp;

CubicSplineInterpolator::CubicSplineInterpolator() {
}

CubicSplineInterpolator::~CubicSplineInterpolator() {
}

RetVal<double> CubicSplineInterpolator::generate_path(
                 const TPQueue& tp_queue ) {
  std::size_t finish_index = tp_queue.size() - 1;
  if ( finish_index <= 0 ) {
    return RetVal<double>( PATH_TOO_SHORT_QUEUE_SIZE, -1.0 );
  }

  if ( g_is_nearEq( tp_queue.get(0).time, 0.0 )
       && g_is_nearEq( tp_queue.get(finish_index).time, 0.0 ) ) {
    return RetVal<double>( PATH_NOT_DEF_100PER_PATH, -1.0 );
  }

  tp_queue_ = tp_queue;
  set_TPVsf( tp_queue_.get(0).time,     tp_queue_.get(finish_index).time,
             tp_queue_.get(0).position, tp_queue_.get(finish_index).position,
             0.0,                       0.0 );

  return RetVal<double>(PATH_SUCCESS, tf_-ts_);
}


RetVal<double> CubicSplineInterpolator::generate_path(
                 const TPVQueue& tpv_queue ) {
  std::size_t finish_index = tpv_queue.size() - 1;
  if ( finish_index <= 0 ) {
    return RetVal<double>( PATH_TOO_SHORT_QUEUE_SIZE, -1.0 );
  }

  if ( g_is_nearEq( tpv_queue.get(0).time, 0.0 )
       && g_is_nearEq( tpv_queue.get(finish_index).time, 0.0 ) ) {
    return RetVal<double>( PATH_NOT_DEF_100PER_PATH, -1.0 );
  }

  tpv_queue_ = tpv_queue;
  set_TPVsf( tpv_queue_.get(0).time,     tpv_queue_.get(finish_index).time,
             tpv_queue_.get(0).position, tpv_queue_.get(finish_index).position,
             tpv_queue_.get(0).velocity, tpv_queue_.get(finish_index).velocity );

  return RetVal<double>(PATH_NOT_DEF_FUNCTION, -1.0);
}
