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
    return RetVal<double>( PATH_INVALID_QUEUE_SIZE, -1.0 );
  }

  if ( g_nearEq( tp_queue.get(finish_index).time, 0.0 ) ) {
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
    return RetVal<double>( PATH_INVALID_QUEUE_SIZE, -1.0 );
  }

  if ( g_nearEq( tpv_queue.get(finish_index).time, 0.0 ) ) {
    return RetVal<double>( PATH_NOT_DEF_100PER_PATH, -1.0 );
  }

  tpv_queue_ = tpv_queue;
  set_TPVsf( tpv_queue_.get(0).time,     tpv_queue_.get(finish_index).time,
             tpv_queue_.get(0).position, tpv_queue_.get(finish_index).position,
             tpv_queue_.get(0).velocity, tpv_queue_.get(finish_index).velocity );

  return RetVal<double>(PATH_NOT_DEF_FUNCTION, -1.0);
}


RetVal<TPV> CubicSplineInterpolator::pop(const double& t ) {
  TPV dest_tpv(t);
  return RetVal<TPV>(PATH_NOT_DEF_FUNCTION, dest_tpv);
}

RetVal<std::vector<double> >
CubicSplineInterpolator::tridiagonal_matrix_eq_solver(
         std::vector<double> d, const std::vector<double>& u,
         const std::vector<double>& l, std::vector<double> p ) {
  if( d.size() != u.size() || d.size() != l.size() || d.size() != p.size() ) {
    THROW( InvalidArgumentSize, " all input parmaeter size must be same." );
  }
  if( d.size() < 1 ) {
    THROW( InvalidArgumentSize, "input parmaeter size must be >= 1." );
  }
  double temp;
  // first loop from top
  for( std::size_t i=0; i<d.size(); i++ ) {
    if( i >= 1 ) {
      temp = l[i] / d[i-1];
      d[i] = d[i] - temp * u[i-1];
      p[i] = p[i] - temp * p[i-1];
    }
    if( g_nearZero(d[i]) ) {
      return RetVal<std::vector<double> >( PATH_INVALID_ARGUMENT_VALUE_ZERO,
                                           std::vector<double>() );
    }
  }
  // solved x
  std::vector<double> x( d.size() );
  std::size_t last_index = d.size()-1;
  x[last_index] = p[last_index] / d[last_index];
  if( d.size() == 1 ) {
    return RetVal<std::vector<double> >( PATH_SUCCESS, x );
  }
  // second loop from bottom
  // list size must be > 2.
  for( int i=last_index-1; i>=0; i-- ) {
    x[i] = ( p[i] - u[i] * x[i+1] ) / d[i];
  }

  return RetVal<std::vector<double> >( PATH_SUCCESS, x );
}
