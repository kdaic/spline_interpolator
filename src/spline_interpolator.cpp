#include "spline_interpolator.hpp"

using namespace interp;

/////////////////////////////////////////////////////////////////////////////////////////

SplineInterpolator::SplineInterpolator() :
  is_path_generated_(false), is_v_limit_(false) {
}

SplineInterpolator::~SplineInterpolator() {
}

const double SplineInterpolator::total_dT() {
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "total dT not exists -- spline-path has not be generated yet.");
  }
  const double out_dT = target_tpva_queue_.get( target_tpva_queue_.size() - 1 ).time
                        - target_tpva_queue_.get( 0 ).time;
  return out_dT;
}

const double SplineInterpolator::v_limit() {
  if( !is_v_limit_ ) {
    THROW( NoVelocityLimit,
           "velocity limits is not defined in this type of path interpolator.");
  }
  return v_limit_;
}

const double SplineInterpolator::finish_time() {
  if( !is_path_generated_ ) {
    THROW( NotSplineGenerated,
           "finish time not exists -- spline-path has not be generated yet.");
  }
  return target_tpva_queue_.get( target_tpva_queue_.size() - 1 ).time;
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

    return generate_path( target_pva_queue );

  } else {

    TPVAQueue target_tpva_queue;
    target_tpva_queue.push_on_clocktime( 0.0, pvas );
    target_tpva_queue.push_on_dT( dT, pvaf );

    return generate_path( target_tpva_queue );
  }

  return SPLINE_NOT_DEF_FUNCTION;
}
