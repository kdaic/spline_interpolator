#include "spline_interpolator.hpp"

using namespace interp;

/////////////////////////////////////////////////////////////////////////////////////////

SplineInterpolator::SplineInterpolator() :
  is_path_generated_(false), is_v_limit_(false) {
}

SplineInterpolator::~SplineInterpolator() {
}

SplineInterpolator::SplineInterpolator( const SplineInterpolator& src ) :
  is_path_generated_ ( src.is_path_generated_ ),
  is_v_limit_        ( src.is_v_limit_        ),
  v_limit_           ( src.v_limit_           ),
  target_tpva_queue_ ( src.target_tpva_queue_ ) {
}


SplineInterpolator::SplineInterpolator(
                      const bool&      is_path_generated,
                      const bool&      is_v_limit,
                      const double&    v_limit,
                      const TPVAQueue& target_tpva_queue ) :
  is_path_generated_ ( is_path_generated ),
  is_v_limit_        ( is_v_limit        ),
  v_limit_           ( v_limit           ),
  target_tpva_queue_ ( target_tpva_queue ) {
}


const double SplineInterpolator::total_dT() const {
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "total dT not exists -- spline-path has not be generated yet.");
  }
  const double out_dT = target_tpva_queue_.get( target_tpva_queue_.size() - 1 ).time
                        - target_tpva_queue_.get( 0 ).time;
  return out_dT;
}

const double SplineInterpolator::v_limit() const {
  if( !is_v_limit_ ) {
    THROW( NoVelocityLimit,
           "velocity limits is not defined in this type of path interpolator.");
  }
  return v_limit_;
}

const double SplineInterpolator::start_time() const {
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "start time not exists -- spline-path has not be generated yet.");
  }
  return target_tpva_queue_.get( 0 ).time;
}

const double SplineInterpolator::finish_time() const {
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "finish time not exists -- spline-path has not be generated yet.");
  }
  return target_tpva_queue_.get( target_tpva_queue_.size() - 1 ).time;
}

const RetCode SplineInterpolator::index_of_time( const double& t,
                                                 std::size_t&  output_index ) const {
  const std::size_t& target_tpva_queue_size = target_tpva_queue_.size();
  const std::size_t last_index = target_tpva_queue_size - 1;
  bool is_out_of_range = true;

  if( t > target_tpva_queue_.get( last_index ).time ) {

    return SPLINE_INVALID_INPUT_TIME;

  } else if( t == target_tpva_queue_.get( last_index ).time ) {

    output_index    = last_index;
    is_out_of_range = false;
    return SPLINE_SUCCESS;
  }
  for ( std::size_t idx=0; idx < target_tpva_queue_size; idx++ ) {

    if( target_tpva_queue_.get( idx ).time <= t
        && t < target_tpva_queue_.get( idx+1 ).time ) {
      output_index    = idx;
      is_out_of_range = false;
      break;
    }
  } // End of for loop idx=0 -> target_tpva_queue_size-1

  if( is_out_of_range ) {
    return SPLINE_INVALID_INPUT_TIME;
  }

  return SPLINE_SUCCESS;
}

RetCode SplineInterpolator::generate_path(
                            const double& ts, const double& tf,
                            const double& xs, const double& xf,
                            const double& vs, const double& vf,
                            const double& as, const double& af ) {

  double dT = tf - ts;

  if( dT < 0.0 ) {
    return SPLINE_INVALID_INPUT_TIME;
  }

  PosVelAcc pvas(xs, vs, as);
  PosVelAcc pvaf(xf, vf, af);
  if( g_isNearlyZero(dT) ) {

    PVAQueue target_pva_queue;
    target_pva_queue.push_back(pvas);
    target_pva_queue.push_back(pvaf);

    return generate_path_from_pva( xs, xf,
                                   vs, vf,
                                   as, af );

  } else {

    TPVAQueue target_tpva_queue;
    target_tpva_queue.push_on_clocktime( 0.0, pvas );
    target_tpva_queue.push_on_dT( dT, pvaf );

    return generate_path( target_tpva_queue );
  }

  return SPLINE_NOT_DEF_FUNCTION;
}


RetCode SplineInterpolator::clear() {

  target_tpva_queue_.clear();

  is_path_generated_ = false;

  return SPLINE_SUCCESS;
}

const std::size_t SplineInterpolator::target_tpva_queue_size() const {
  return target_tpva_queue_.size();
}
