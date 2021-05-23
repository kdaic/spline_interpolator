#include "cubic_spline_interpolator.hpp"

using namespace interp;

CubicSplineInterpolator::CubicSplineInterpolator() {
}

CubicSplineInterpolator::~CubicSplineInterpolator() {
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path(
          const TPQueue& target_tp_queue,
          const double vs, const double vf,
          const double as, const double af) {
  std::size_t finish_index = target_tp_queue.size() - 1;
  if ( finish_index <= 1 ) {
    return SPLINE_INVALID_QUEUE_SIZE;
  }
  //
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
    if( g_isNearlyZero( target_tp_queue.dT(i) ) ) {
      // Failed because dT = 0
      return SPLINE_INVALID_INPUT_INTERVAL_TIME_DT;
    }
    inverse_dT     = 1.0 / target_tp_queue.dT(i);
    inverse_pre_dT = 1.0 / target_tp_queue.dT(i-1);
    lower.push_back( 2.0 * inverse_dT );
    diago.push_back( 4.0 * (inverse_dT + inverse_pre_dT) );
    upper.push_back( 2.0 * inverse_dT );
    double p
      = 6.0*(target_tp_queue.get(i+1).value - target_tp_queue.get(i).value)
           * inverse_dT * inverse_dT
      + 6.0*(target_tp_queue.get(i).value - target_tp_queue.get(i-1).value)
           * inverse_pre_dT * inverse_pre_dT;
    param.push_back(p);
  }
  // the finish index
  lower.push_back(0.0);
  diago.push_back(1.0);
  upper.push_back(0.0);
  param.push_back( vf ); // this corresponds to finish velocity.
  std::vector<double> ret_c;
  RetCode retcode = tridiagonal_matrix_eq_solver( diago, upper, lower, param, ret_c );
  if( retcode != SPLINE_SUCCESS ) {
    // diago[i]=0, SPLINE_INVALID_MATRIX_ARGUMENT_VALUE_ZERO
    return retcode;
  }
  //
  c_.clear();
  c_.resize( ret_c.size() );
  std::copy( ret_c.begin(), ret_c.end(), c_.begin() );
  //
  a_.clear();
  b_.clear();
  d_.clear();
  target_tpva_queue_.clear();
  // the start index = 0
  for ( std::size_t i=0; i < finish_index; i++ ) {
    inverse_dT = 1.0 / target_tp_queue.dT(i);
    //
    double dp = target_tp_queue.get(i+1).value - target_tp_queue.get(i).value;
    //
    a_.push_back( ( (c_[i+1] + c_[i]) * target_tp_queue.dT(i) - 2.0 * dp )
                  * inverse_dT * inverse_dT * inverse_dT );
    b_.push_back( ( -1.0 * (c_[i+1] + 2.0 * c_[i]) * target_tp_queue.dT(i) + 3.0 * dp )
                  * inverse_dT * inverse_dT );
    d_.push_back( target_tp_queue.get(i).value );
    //
    target_tpva_queue_.push( target_tp_queue.get(i).time,
                             target_tp_queue.get(i).value,
                             c_[i],
                             b_[i] );
    // std::cout << "a_[" << i << "] : " << a_[i] << std::endl;
    // std::cout << "b_[" << i << "] : " << b_[i] << std::endl;
    // std::cout << "c_[" << i << "] : " << c_[i] << std::endl;
    // std::cout << "d_[" << i << "] : " << d_[i] << std::endl;
  }
  // the finish index = target_tp_queue.size() - 1
  a_.push_back( 0.0 ); // this corresponds to finish jark :=0.0.
  b_.push_back( 0.0 ); // this corresponds to finish velocity :=0.0.
  d_.push_back( target_tp_queue.get(finish_index).value ); // this corresponds to finish position.
  // std::cout << "a_[" << finish_index << "] : " << a_[finish_index] << std::endl;
  // std::cout << "b_[" << finish_index << "] : " << b_[finish_index] << std::endl;
  // std::cout << "c_[" << finish_index << "] : " << c_[finish_index] << std::endl;
  // std::cout << "d_[" << finish_index << "] : " << d_[finish_index] << std::endl;
  TimePVA finish_tpva( target_tp_queue.get(finish_index).time,
                       PosVelAcc( target_tp_queue.get(finish_index).value,
                                  c_[finish_index],
                                  b_[finish_index] ) );
  target_tpva_queue_.push( finish_tpva );
  //
  is_path_generated_ = true;
  //
  return SPLINE_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path(
                 const TPVAQueue& target_tpva_queue ) {
  std::size_t finish_index = target_tpva_queue.size() - 1;
  if ( finish_index <= 0 ) {
    return SPLINE_INVALID_QUEUE_SIZE;
  }
  //
  if ( g_isNearlyEq( target_tpva_queue.get(finish_index).time, 0.0 ) ) {
    return SPLINE_NOT_DEF_100PER_PATH;
  }
  //
  target_tpva_queue_ = target_tpva_queue;
  //
  a_.clear();
  b_.clear();
  c_.clear();
  d_.clear();
  for( std::size_t i=0; i < finish_index; i++ ) {
    const double& pos1 = target_tpva_queue_.get( i+1 ).value.pos;
    const double& pos0 = target_tpva_queue_.get( i   ).value.pos;
    const double& vel1 = target_tpva_queue_.get( i+1 ).value.vel;
    const double& vel0 = target_tpva_queue_.get( i   ).value.vel;
    const double  dT0  = target_tpva_queue_.dT(  i   );
    a_.push_back( ((vel1 + vel0)*dT0     - 2.0*(pos1 - pos0)) / (dT0 * dT0 * dT0) );
    b_.push_back( ((vel1 + 2.0*vel0)*dT0 + 3.0*(pos1 - pos0)) / (dT0 * dT0 ) );
    c_.push_back( vel0 );
    d_.push_back( pos0 );
  }
  a_.push_back( 0.0 ); // this corresponds to finish jark :=0.0.
  // this corresponds to finish acceleration.
  b_.push_back( target_tpva_queue_.get( finish_index ).value.acc * 0.5 );
  c_.push_back( target_tpva_queue_.get( finish_index ).value.vel );
  d_.push_back( target_tpva_queue_.get( finish_index ).value.pos );
  //
  is_path_generated_ = true;
  //
  return SPLINE_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::generate_path_from_pva(
                                 const double& xs, const double& xf,
                                 const double& vs, const double& vf,
                                 const double& as, const double& af) {
  THROW( UndefSplineException,
         "CubicSplineInterpolator not supports this type - 100 per path generation" );
  return SPLINE_NOT_DEF_100PER_PATH;
}

/////////////////////////////////////////////////////////////////////////////////////////////

const TimePVA CubicSplineInterpolator::pop(const double& t ) {
  TimePVA empty_tpva(t);
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "pop data not exists -- Path has not be generated yet.");
  }
  int index= -1;
  for ( std::size_t i=0; i < target_tpva_queue_.size(); i++ ) {
    if( target_tpva_queue_.get(i).time <= t  && t <= target_tpva_queue_.get(i+1).time ) {
      index = i;
      break;
    }
  }
  if( index == -1 ) {
    THROW( TimeOutOfRange,
           "time is out of range of generated path" );
  }
  //
  double dTi    = (t - target_tpva_queue_.get(index).time);
  double square = dTi * dTi;
  double cube   = dTi * dTi * dTi;
  double position = a_[index] * cube + b_[index] * square + c_[index] * dTi + d_[index];
  double velocity = 3.0 * a_[index] * square + 2.0 * b_[index] * dTi + c_[index];
  double acceleration = 6.0 * a_[index] * dTi + 2.0 * b_[index];
  TimePVA dest_tpva( t,
                     PosVelAcc(position, velocity, acceleration) );
  //
  return dest_tpva;
}

/////////////////////////////////////////////////////////////////////////////////////////////

RetCode CubicSplineInterpolator::tridiagonal_matrix_eq_solver(
          std::vector<double> d, const std::vector<double>& u,
          const std::vector<double>& l, std::vector<double> p,
          std::vector<double>& out_solved_x ) {
  out_solved_x.clear();
  if( d.size() != u.size() || d.size() != l.size() || d.size() != p.size() ) {
    THROW( InvalidArgumentSize, "all input parmeter size must be same." );
  }
  if( d.size() < 1 ) {
    THROW( InvalidArgumentSize, "input parmeter size must be >= 1." );
  }
  double temp;
  // first loop from top
  for( std::size_t i=0; i<d.size(); i++ ) {
    if( g_isNearlyZero(d[i]) ) {
      return SPLINE_INVALID_MATRIX_ARGUMENT_VALUE_ZERO;
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
    return SPLINE_SUCCESS;
  }
  // second loop from bottom
  // list size must be >= 2.
  for( int i=last_index-1; i>=0; i-- ) {
    out_solved_x[i] = ( p[i] - u[i] * out_solved_x[i+1] ) / d[i];
  }
  //
  return SPLINE_SUCCESS;
}
