#include "non_uniform_rounding_spline.hpp"

using namespace interp;

NonUniformRoundingSpline::NonUniformRoundingSpline() {
}

NonUniformRoundingSpline::NonUniformRoundingSpline(
                           const NonUniformRoundingSpline& src ) :
  tp_buffer_ ( src.tp_buffer_  ),
  tpva_buffer_( src.tpva_buffer_ )
{
}


NonUniformRoundingSpline::NonUniformRoundingSpline( const double& init_position,
                                                    const double& init_velocity ) {
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

NonUniformRoundingSpline NonUniformRoundingSpline::operator=(
                           const NonUniformRoundingSpline& src
                         ) {
  NonUniformRoundingSpline dest(src);
  tp_buffer_   = dest.tp_buffer_;
  tpva_buffer_ = dest.tpva_buffer_;
  return *this;
}

double NonUniformRoundingSpline::calculate_velocity( TimePosition tp0,
                                                     TimePosition tp1,
                                                     TimePosition tp2 ) {
  double t0 = tp0.time;
  double x0 = tp0.value; // position

  double t1 = tp1.time;
  double x1 = tp1.value; // position

  double t2 = tp2.time;
  double x2 = tp2.value; // position

  if ((t1 - t0 <= 0.0) || (t2 - t1 <= 0.0)) {
    THROW( InvalidArgumentValue, "wrong time direction." );
  }

  double dist01 = sqrt( (t1 - t0)*(t1 - t0) + (x1 - x0)*(x1 - x0) );
  double dist12 = sqrt( (t2 - t1)*(t2 - t1) + (x2 - x1)*(x2 - x1) );
  double dt = (t2 - t1) / dist12 - (t0 - t1) / dist01;
  double dx = (x2 - x1) / dist12 - (x0 - x1) / dist01;

  // std::cout << "\n\n ==== Non-Uiform Spline Velocity ====\n";
  // std::cout << "dt : " << dt << std::endl;
  // std::cout << "dx : " << dx << std::endl;
  // std::cout << "v  : " << dx/dt << std::endl;

  if (dt == 0.0) {
    std::out_of_range("velocity is infinity (dt = 0.0).");
  }

  return dx / dt;
}


void NonUniformRoundingSpline::push_with_velocity_no_change(
                                 const double& clock_time,
                                 const double& position ) {
  TimePosition tp( clock_time, position );
  tp_buffer_.push(tp);

  TimePVA init_tpv( tp.time,
                    PosVelAcc( tp.value, // position
                               0.0,
                               0.0 ) );
  tpva_buffer_.push(init_tpv);
  if ( tp_buffer_.size() >= 3) {
    // delete front(oldest) data
    tp_buffer_.pop_delete();
  }
}


void NonUniformRoundingSpline::push_with_velocity_no_change (
                                 const TimePVA& time_pva ) {
  TimePosition tp( time_pva.time, time_pva.P.pos);
  tp_buffer_.push(tp);

  tpva_buffer_.push(time_pva);
  if ( tp_buffer_.size() >= 3) {
    // delete front(oldest) data
    tp_buffer_.pop_delete();
  }
}


void NonUniformRoundingSpline::push( const double& clock_time, const double& position ) {

  TimePosition tp( clock_time, position );
  tp_buffer_.push(tp);

  TimePVA init_tpv( tp.time,
                    PosVelAcc( tp.value, // position
                               0.0,
                               0.0 ) );
  tpva_buffer_.push(init_tpv);

  if ( tp_buffer_.size() >= 3) {
    TimePVA tpva( tp_buffer_.get( tp_buffer_.size()-2 ).time,
                  PosVelAcc( tp_buffer_.get( tp_buffer_.size()-2 ).value, // position
                             this->calculate_velocity( tp_buffer_.get(0),
                                                       tp_buffer_.get(1),
                                                       tp_buffer_.get(2) ),
                             0.0 ) );
    tpva_buffer_.set( tpva_buffer_.size()-2, tpva);
    // delete front(oldest) data
    tp_buffer_.pop_delete();
  }
}


void NonUniformRoundingSpline::push( const TimePVA& time_pva ) {

  TimePosition tp( time_pva.time, time_pva.P.pos);
  tp_buffer_.push(tp);

  tpva_buffer_.push(time_pva);

  if ( tp_buffer_.size() >= 3) {
    TimePVA tpva( tp_buffer_.get( tp_buffer_.size()-2 ).time,
                  PosVelAcc( tp_buffer_.get( tp_buffer_.size()-2 ).value, // position
                             this->calculate_velocity( tp_buffer_.get(0),
                                                       tp_buffer_.get(1),
                                                       tp_buffer_.get(2) ),
                             0.0 ) );
    tpva_buffer_.set( tpva_buffer_.size()-2, tpva);
    // delete front(oldest) data
    tp_buffer_.pop_delete();
  }
}


void NonUniformRoundingSpline::push_dT_with_velocity_no_change(
                                 const double& interval_time, const double& position) {
  double pre_time = 0.0;
  if( tp_buffer_.size() > 0 ) {
    pre_time = tp_buffer_.back().time;
  }
  this->push_with_velocity_no_change( pre_time + interval_time, position );
}


void NonUniformRoundingSpline::push_dT(const double& interval_time, const double& position) {
  double pre_time = 0.0;
  if( tp_buffer_.size() > 0 ) {
    pre_time = tp_buffer_.back().time;
  }
  this->push( pre_time + interval_time, position );
}


const TPVAQueue NonUniformRoundingSpline::tpva_queue() const {
  return tpva_buffer_;
}

const TimePVA NonUniformRoundingSpline::get( const std::size_t& index ) const {
  return tpva_buffer_.get(index);
}

const TimePVA NonUniformRoundingSpline::pop() {
  return tpva_buffer_.pop();
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

const double NonUniformRoundingSpline::dT( const std::size_t& index ) const
  throw(InvalidIndexAccess) {

  return tpva_buffer_.dT(index);
}

const double NonUniformRoundingSpline::total_dT() const {
  return tpva_buffer_.total_dT();
}


