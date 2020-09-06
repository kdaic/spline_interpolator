#include "cubic_spline_interpolator.hpp"
#include <plog/Log.h>

using namespace interp;


CubicSplineInterpolator::CubicSplineInterpolator() {
}

CubicSplineInterpolator::~CubicSplineInterpolator() {
}

RetCode CubicSplineInterpolator::generate_path(
          const TPQueue& tp_queue,
          const double vs, const double vf) {
  std::size_t finish_index = tp_queue.size() - 1;
  if ( finish_index <= 1 ) {
    return PATH_INVALID_QUEUE_SIZE;
  }

  std::vector<double> upper;
  std::vector<double> diago;
  std::vector<double> lower;
  std::vector<double> param;
  // the start index = 0
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back( vs ); // this corresponds to start velocity.
  // index >= 1
  double inverse_dT = 0.0;
  double inverse_pre_dT = 0.0;
  for ( std::size_t i=1; i < finish_index; i++ ) {
    if( g_nearZero( tp_queue.dT(i) ) ) {
      // Failed because dT = 0
      return PATH_INVALID_INPUT_INTERVAL_TIME_DT;
    }
    inverse_dT = 1.0 / tp_queue.dT(i);
    inverse_pre_dT = 1.0 / tp_queue.dT(i-1);
    lower.push_back( 2.0 * inverse_dT );
    diago.push_back( 4.0 * (inverse_dT + inverse_pre_dT) );
    upper.push_back( 2.0 * inverse_dT );
    double p
      = 6.0*(tp_queue.get(i+1).position - tp_queue.get(i).position) * inverse_dT * inverse_dT
      + 6.0*(tp_queue.get(i).position - tp_queue.get(i-1).position) * inverse_pre_dT * inverse_pre_dT;
    param.push_back(p);
  }
  // the finish index
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back( vf ); // this corresponds to finish velocity.
  std::vector<double> ret_c;
  RetCode retcode = tridiagonal_matrix_eq_solver( diago, upper, lower, param, ret_c );
  if( retcode != PATH_SUCCESS ) {
    // diago[i]=0, PATH_INVALID_MATRIX_ARGUMENT_VALUE_ZERO
    return retcode;
  }

  LOGD << "solver result :" << (retcode == PATH_SUCCESS);

  c_.clear();
  c_.resize( ret_c.size() );
  std::copy( ret_c.begin(), ret_c.end(), c_.begin() );

  a_.clear();
  b_.clear();
  d_.clear();
  tpv_queue_.clear();
  // the start index = 0
  for ( std::size_t i=0; i < finish_index; i++ ) {
    inverse_dT = 1.0 / tp_queue.dT(i);

    double dp = tp_queue.get(i+1).position - tp_queue.get(i).position;

    a_.push_back( ( (c_[i+1] + c_[i]) * tp_queue.dT(i) - 2.0 * dp )
                  * inverse_dT * inverse_dT * inverse_dT );
    b_.push_back( ( -1.0 * (c_[i+1] + 2.0 * c_[i]) * tp_queue.dT(i) + 3.0 * dp )
                  * inverse_dT * inverse_dT );
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
  a_.push_back( 0.0 ); // this corresponds to finish jark :=0.0.
  b_.push_back( 0.0 ); // this corresponds to finish velocity :=0.0.
  d_.push_back( tp_queue.get(finish_index).position ); // this corresponds to finish position.
  LOGD << "a_[" << finish_index << "] : " << a_[finish_index];
  LOGD << "b_[" << finish_index << "] : " << b_[finish_index];
  LOGD << "c_[" << finish_index << "] : " << c_[finish_index];
  LOGD << "d_[" << finish_index << "] : " << d_[finish_index];
  TPV finish_tpv( tp_queue.get(finish_index).time,
                  tp_queue.get(finish_index).position,
                  c_[finish_index] );
  tpv_queue_.push( finish_tpv );

  is_path_generated_ = true;

  return PATH_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path_acc(
          const TPQueue& tp_queue,
          const double as, const double af) {
  std::size_t finish_index = tp_queue.size() - 1;
  if ( finish_index <= 1 ) {
    return PATH_INVALID_QUEUE_SIZE;
  }

  std::vector<double> upper;
  std::vector<double> diago;
  std::vector<double> lower;
  std::vector<double> param;
  // the start index = 0
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back( as ); // this corresponds start acceleration :=0.0.
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
  param.push_back( af ); // this corresponds to finish acceleration :=0.0.
  std::vector<double> ret_b;
  RetCode retcode = tridiagonal_matrix_eq_solver( diago, upper, lower, param, ret_b );
  if( retcode != PATH_SUCCESS ) {
    // diago[i]=0, PATH_INVALID_MATRIX_ARGUMENT_VALUE_ZERO
    return retcode;
  }

  b_.clear();
  b_.resize( ret_b.size() );
  std::copy( ret_b.begin(), ret_b.end(), b_.begin() );

  a_.clear();
  c_.clear();
  d_.clear();
  // the start index = 0
  for ( std::size_t i=0; i < finish_index; i++ ) {
    a_.push_back( (b_[i+1] - b_[i]) / (3.0*tp_queue.dT(i)) );
    c_.push_back( (tp_queue.get(i+1).position - tp_queue.get(i).position)
                    / tp_queue.dT(i)
                  - tp_queue.dT(i) * (b_[i+1] + 2.0*b_[i]) / 3.0 );
    d_.push_back( tp_queue.get(i).position );
    TPV tpv( tp_queue.get(i).time,
             tp_queue.get(i).position,
             c_[i] );
    tpv_queue_.push( tpv );
  }
  a_.push_back( 0.0 ); // this corresponds to finish jark :=0.0.
  c_.push_back( 0.0 ); // this corresponds to finish velocity :=0.0.
  d_.push_back( tp_queue.get(finish_index).position ); // this corresponds finish position.
  TPV finish_tpv( tp_queue.get(finish_index).time,
                  tp_queue.get(finish_index).position,
                  c_[finish_index] );
  tpv_queue_.push( finish_tpv );

  is_path_generated_ = true;

  return PATH_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path(
          const TPVQueue& tpv_queue ) {
  std::size_t finish_index = tpv_queue.size() - 1;
  if ( finish_index <= 0 ) {
    return PATH_INVALID_QUEUE_SIZE;
  }

  if ( g_nearEq( tpv_queue.get(finish_index).time, 0.0 ) ) {
    return PATH_NOT_DEF_100PER_PATH;
  }

  tpv_queue_ = tpv_queue;

  return PATH_NOT_DEF_FUNCTION;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path(
          const PVQueue& pv_queue ) {
  THROW( UndefPathException,
         "CubicSplineInterpolator not supports this type - 100 per path generation" );
  return PATH_NOT_DEF_100PER_PATH;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::pop(const double& t, TPV& output ) {
  TPV empty_tpv(t);
  if( !is_path_generated_ ) {
    output = empty_tpv;
    return PATH_NOT_GENERATED;
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
    output = empty_tpv;
    return PATH_TIME_IS_OUT_OF_RANGE;
  }

  //
  double dTi    = (t - tpv_queue_.get(index).time);
  double square = dTi * dTi;
  double cube   = dTi * dTi * dTi;
  double position = a_[index] * cube + b_[index] * square + c_[index] * dTi + d_[index];
  double velocity = 3.0 * a_[index] * square + 2.0 * b_[index] * dTi + c_[index];
  TPV dest_tpv(t, position, velocity);
  //
  output = dest_tpv;
  return PATH_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::tridiagonal_matrix_eq_solver(
          std::vector<double> d, const std::vector<double>& u,
          const std::vector<double>& l, std::vector<double> p,
          std::vector<double>& out_solved_x ) {
  out_solved_x.clear();
  if( d.size() != u.size() || d.size() != l.size() || d.size() != p.size() ) {
    THROW( InvalidArgumentSize, " all input parmaeter size must be same." );
  }
  if( d.size() < 1 ) {
    THROW( InvalidArgumentSize, "input parmaeter size must be >= 1." );
  }
  double temp;
  // first loop from top
  for( std::size_t i=0; i<d.size(); i++ ) {
    if( g_nearZero(d[i]) ) {
      return PATH_INVALID_MATRIX_ARGUMENT_VALUE_ZERO;
    }
    if( i >= 1 ) {
      temp = l[i] / d[i-1];
      d[i] = d[i] - temp * u[i-1];
      p[i] = p[i] - temp * p[i-1];
    }
  }
  //
  // solve x
  //
  out_solved_x.resize( d.size() );
  std::size_t last_index = d.size()-1;
  out_solved_x[last_index] = p[last_index] / d[last_index];
  if( d.size() == 1 ) {
    return PATH_SUCCESS;
  }
  // second loop from bottom
  // list size must be > 2.
  for( int i=last_index-1; i>=0; i-- ) {
    out_solved_x[i] = ( p[i] - u[i] * out_solved_x[i+1] ) / d[i];
  }

  return PATH_SUCCESS;
}
