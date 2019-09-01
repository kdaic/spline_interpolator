#include "cubic_spline_interpolator.hpp"
#include <plog/Log.h>

using namespace interp;


CubicSplineInterpolator::CubicSplineInterpolator() {
}

CubicSplineInterpolator::~CubicSplineInterpolator() {
}

RetVal<double> CubicSplineInterpolator::generate_path(
                 const TPQueue& tp_queue ) {
  std::size_t finish_index = tp_queue.size() - 1;
  if ( finish_index <= 1 ) {
    return RetVal<double>( PATH_INVALID_QUEUE_SIZE, -1.0 );
  }

  std::vector<double> upper;
  std::vector<double> diago;
  std::vector<double> lower;
  std::vector<double> param;
  // the start index = 0
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back(0.0); // this corresponds start acceleration :=0.0.
  // index >= 1
  for ( std::size_t i=1; i < finish_index; i++ ) {
    lower.push_back( tp_queue.dT(i-1) );
    diago.push_back( 2 * (tp_queue.dT(i-1) + tp_queue.dT(i)) );
    upper.push_back( tp_queue.dT(i) );
    double p
      = 3.0*(tp_queue.get(i+1).position - tp_queue.get(i).position) / tp_queue.dT(i)
      - 3.0*(tp_queue.get(i).position - tp_queue.get(i-1).position) / tp_queue.dT(i-1);
    param.push_back(p);
  }
  // the finish index = tp_queue.size()-1
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back(0.0); // this corresponds finish acceleration :=0.0.
  RetVal<std::vector<double> > ret_b
    = tridiagonal_matrix_eq_solver( diago, upper, lower, param );

  b_.clear();
  b_.resize( ret_b.value.size() );
  std::copy( ret_b.value.begin(), ret_b.value.end(), b_.begin() );

  a_.clear();
  c_.clear();
  d_.clear();
  // the start index = 0
  // a_.push_back( 0.0 ); // this corresponds start jark :=0.0.
  // c_.push_back( 0.0 ); // this corresponds start velocity :=0.0.
  // d_.push_back( tp_queue.get(0).position ); // this corresponds start position.
  // index >= 1
  for ( std::size_t i=0; i < finish_index; i++ ) {
    a_.push_back( (b_[i+1] - b_[i]) / (3.0*tp_queue.dT(i)) );
    c_.push_back( (tp_queue.get(i+1).position - tp_queue.get(i).position) / tp_queue.dT(i)
                  - tp_queue.dT(i) * (b_[i+1] + 2.0*b_[i]) / 3.0 );
    d_.push_back( tp_queue.get(i).position );
    TPV tpv( tp_queue.get(i).time,
             tp_queue.get(i).position,
             c_[i] );
    tpv_queue_.push( tpv );
    LOGD << "a_[" << i << "] : " << a_[i];
    LOGD << "b_[" << i << "] : " << b_[i];
    LOGD << "c_[" << i << "] : " << c_[i];
    LOGD << "d_[" << i << "] : " << d_[i];
  }
  // the finish index = tp_queue.size() - 1
  a_.push_back( 0.0 ); // this corresponds finish jark :=0.0.
  c_.push_back( 0.0 ); // this corresponds finish velocity :=0.0.
  d_.push_back( tp_queue.get(finish_index).position ); // this corresponds finish position.
  TPV finish_tpv( tp_queue.get(finish_index).time,
                  tp_queue.get(finish_index).position,
                  c_[finish_index] );
  tpv_queue_.push( finish_tpv );

  is_path_generated_ = true;

  return total_dT();
}

/////////////////////////////////////////////////////////////////////////////////////////////

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

  return RetVal<double>(PATH_NOT_DEF_FUNCTION, -1.0);
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetVal<double> CubicSplineInterpolator::generate_path(
                 const PVQueue& pv_queue ) {
  THROW( UndefPathException, "CubicSplineInterpolator not supports this type - 100 per path generation" );
  return RetVal<double>( PATH_NOT_DEF_100PER_PATH, -1.0 );
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetVal<TPV> CubicSplineInterpolator::pop(const double& t ) {
  TPV empty_tpv(t);
  if( !is_path_generated_ ) {
    return RetVal<TPV>(PATH_NOT_GENERATED, empty_tpv);
  }
  int index= -1;
  for ( std::size_t i=0; i < tpv_queue_.size(); i++ ) {
    if( tpv_queue_.get(i).time <= t  && t <= tpv_queue_.get(i+1).time ) {
      index = i;
      break;
    }
  }
  if( index == -1 ) {
    // time is not within the range of generated path
    return RetVal<TPV>(PATH_TIME_IS_OUT_OF_RANGE, empty_tpv);
  }

  //
  double dTi    = (t - tpv_queue_.get(index).time);
  double square = dTi * dTi;
  double cube   = dTi * dTi * dTi;
  double position = a_[index] * cube + b_[index] * square + c_[index] * dTi + d_[index];
  double velocity = 3.0 * a_[index] * square + 2.0 * b_[index] * dTi + c_[index];
  TPV dest_tpv(t, position, velocity);
  //
  return RetVal<TPV>(PATH_SUCCESS, dest_tpv);
}

/////////////////////////////////////////////////////////////////////////////////////////////

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
  //
  // solve x
  //
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
