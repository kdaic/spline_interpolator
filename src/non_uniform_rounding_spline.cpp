#include "non_uniform_rounding_spline.hpp"

using namespace interp;

NonUniformRoundingSpline::NonUniformRoundingSpline(const double& init_position,
                                                   const double& init_velocity) {
  TimePosition tp(0.0, init_position);
  tp_buffer_.push(tp);

  TimePVA tpv( tp.time,
               PosVelAcc( tp.value, // position
                          init_velocity,
                          0.0 ) );
  tpva_buffer_.push(tpv);
}

NonUniformRoundingSpline::~NonUniformRoundingSpline() {
  this->clear();
};


void NonUniformRoundingSpline::push(const double& time, const double& position) {
  TimePosition tp( tp_buffer_.back().time + time, position );
  tp_buffer_.push(tp);

  TimePVA init_tpv(tp.time,
                   PosVelAcc( tp.value, // position
                              0.0,
                              0.0 ) );
  tpva_buffer_.push(init_tpv);

  if ( tp_buffer_.size() >= 3) {
    TimePVA tpva( tp_buffer_.get( tp_buffer_.size()-2 ).time,
                  PosVelAcc( tp_buffer_.get( tp_buffer_.size()-2 ).value, // position
                             this->calculate_velocity(),
                             0.0 ) );
    tpva_buffer_.set( tpva_buffer_.size()-2, tpva);
    // delete front(oldest) data
    tp_buffer_.pop_delete();
  }
}

double NonUniformRoundingSpline::calculate_velocity() {
  double t0 = tp_buffer_.get(0).time;
  double x0 = tp_buffer_.get(0).value; // position

  double t1 = tp_buffer_.get(1).time;
  double x1 = tp_buffer_.get(1).value; // position

  double t2 = tp_buffer_.get(2).time;
  double x2 = tp_buffer_.get(2).value; // position

  if ((t1 - t0 <= 0.0) || (t2 - t1 <= 0.0)) {
    THROW( InvalidArgumentValue, "wrong time direction." );
  }

  double dist01 = sqrt( (t1 - t0)*(t1 - t0) + (x1 - x0)*(x1 - x0) );
  double dist12 = sqrt( (t2 - t1)*(t2 - t1) + (x2 - x1)*(x2 - x1) );
  double dt = (t2 - t1) / dist12 - (t0 - t1) / dist01;
  double dx = (x2 - x1) / dist12 - (x0 - x1) / dist01;

  LOGD << "\n\n ==== Non-Uiform Spline Velocity ====\n";
  LOGD << "dt : " << dt << std::endl;
  LOGD << "dx : " << dx << std::endl;
  LOGD << "v  : " << dx/dt << std::endl;

  if (dt == 0.0) {
    std::out_of_range("velocity is infinity (dt = 0.0).");
  }

  return dx / dt;
}

RetCode NonUniformRoundingSpline::pop(TimePVA& output) {
  TimeVal<PosVelAcc> tmp_output;
  RetCode ret = tpva_buffer_.pop(tmp_output);
  if( ret == PATH_SUCCESS )
  {
    output = tmp_output;
  }
  return ret;
}

void NonUniformRoundingSpline::clear() {
  if ( tp_buffer_.size() != 0 ) {
    tp_buffer_.clear();
  }
  if ( tpva_buffer_.size() != 0 ) {
    tpva_buffer_.clear();
  }
}

unsigned int NonUniformRoundingSpline::size() {
  return tpva_buffer_.size();
}
