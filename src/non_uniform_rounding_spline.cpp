#include "non_uniform_rounding_spline.hpp"

using namespace interp;

NonUniformRoundingSpline::NonUniformRoundingSpline() {
}

NonUniformRoundingSpline::NonUniformRoundingSpline(
                           const NonUniformRoundingSpline& src ) :
  tpva_buffer_( src.tpva_buffer_ )
{
}


NonUniformRoundingSpline::NonUniformRoundingSpline( const double& init_position,
                                                    const double& init_velocity ) {

  PosVelAcc pva( init_position, // position
                 init_velocity, // velocity
                 0.0 );         // acceleration
  TimePVA tpv( 0.0,   // time
               pva ); // PosVelAcc
  tpva_buffer_.push(tpv);
}

NonUniformRoundingSpline::~NonUniformRoundingSpline() {
  this->clear();
};

NonUniformRoundingSpline NonUniformRoundingSpline::operator=(
                           const NonUniformRoundingSpline& src
                         ) {
  NonUniformRoundingSpline dest(src);
  tpva_buffer_ = dest.tpva_buffer_;
  return *this;
}

double NonUniformRoundingSpline::calculate_velocity( const TimePVA& tp0,
                                                     const TimePVA& tp1,
                                                     const TimePVA& tp2 ) {
  double t0 = tp0.time;
  double x0 = tp0.P.pos; // position

  double t1 = tp1.time;
  double x1 = tp1.P.pos; // position

  double t2 = tp2.time;
  double x2 = tp2.P.pos; // position

  if ((t1 - t0 <= 0.0) || (t2 - t1 <= 0.0)) {
    const std::string err_msg = "wrong time direction.";
    std::cerr << err_msg << std::endl;
    THROW( InvalidArgumentValue, err_msg );
  }

  double distance_01 = sqrt( (t1 - t0)*(t1 - t0) + (x1 - x0)*(x1 - x0) );
  double distance_12 = sqrt( (t2 - t1)*(t2 - t1) + (x2 - x1)*(x2 - x1) );
  double dt = (t2 - t1) / distance_12 - (t0 - t1) / distance_01;
  double dx = (x2 - x1) / distance_12 - (x0 - x1) / distance_01;

  // std::cout << "\n\n ==== Non-Uiform Spline Velocity ====\n";
  // std::cout << "dt : " << dt << std::endl;
  // std::cout << "dx : " << dx << std::endl;
  // std::cout << "v  : " << dx/dt << std::endl;

  if (dt == 0.0) {
    const std::string err_msg = "velocity is infinity (dt = 0.0).";
    std::cerr << err_msg << std::endl;
    THROW( TimeOutOfRange, err_msg );
  }

  return dx / dt;
}


RetCode NonUniformRoundingSpline::push_without_velocity_change(
                                    const double& clock_time,
                                    const double& position ) {
  PosVelAcc pva( position, // position
                 0.0,      // velocity
                 0.0 );    // acceleration
  TimePVA init_tpv( clock_time, pva );
  return tpva_buffer_.push( init_tpv );
}


RetCode NonUniformRoundingSpline::push_without_velocity_change (
                                    const TimePVA& time_pva ) {
  return tpva_buffer_.push(time_pva);
}


RetCode NonUniformRoundingSpline::push( const double& clock_time,
                                        const double& position ) {

  PosVelAcc init_pva( position, // position
                      0.0,      // initial velocity
                      0.0 );    // initial acceleration
  TimePVA init_tpv( clock_time, // time
                    init_pva ); // PosVelAcc

  RetCode ret_push = tpva_buffer_.push(init_tpv);

  if( ret_push != SPLINE_SUCCESS ) {
    return ret_push;
  }

  const unsigned int buffer_size = tpva_buffer_.size();

  if ( buffer_size >= 3 ) {
    PosVelAcc pva( tpva_buffer_.get( buffer_size-2 ).P.pos,     // position
                   this->calculate_velocity(
                           tpva_buffer_.get( buffer_size-3 ),
                           tpva_buffer_.get( buffer_size-2 ),
                           tpva_buffer_.get( buffer_size-1 ) ), // velocity
                   0.0 );                                       // acceleration

    TimePVA tpva( tpva_buffer_.get( buffer_size-2 ).time,       // time
                  pva );                                        // PosVelAcc

    RetCode ret_set = tpva_buffer_.set( buffer_size-2, tpva );

    if( ret_set != SPLINE_SUCCESS ) {
      return ret_set;
    }

  } // End of if( buffer_size_>=3 )

  return SPLINE_SUCCESS;
}


RetCode NonUniformRoundingSpline::push( const TimePVA& time_pva ) {

  tpva_buffer_.push(time_pva);

  const unsigned int buffer_size = tpva_buffer_.size();

  if ( buffer_size >= 3 ) {
    PosVelAcc pva( tpva_buffer_.get( buffer_size-2 ).P.pos,     // position
                   this->calculate_velocity(
                           tpva_buffer_.get( buffer_size-3 ),
                           tpva_buffer_.get( buffer_size-2 ),
                           tpva_buffer_.get( buffer_size-1 ) ), // velocity
                   0.0 );                                       // acceleration

    TimePVA tpva( tpva_buffer_.get( buffer_size-2 ).time,       // time
                  pva );                                        // PosVelAcc

    RetCode ret_set = tpva_buffer_.set( buffer_size-2, tpva );

    if( ret_set != SPLINE_SUCCESS ) {
      return ret_set;
    }

  } // End of if( buffer_size_>=3 )

  return SPLINE_SUCCESS;
}


RetCode NonUniformRoundingSpline::push_dT_without_velocity_change(
                                    const double& interval_time,
                                    const double& position ) {
  double pre_time = 0.0;
  if( tpva_buffer_.size() > 0 ) {
    pre_time = tpva_buffer_.back().time;
  }
  return this->push_without_velocity_change( pre_time + interval_time, // time
                                             position );               // position
}

RetCode NonUniformRoundingSpline::push_dT_without_velocity_change(
                                    const double& interval_time,
                                    const PosVelAcc& pva ) {
  double pre_time = 0.0;
  if( tpva_buffer_.size() > 0 ) {
    pre_time = tpva_buffer_.back().time;
  }

  TimePVA tpva( pre_time + interval_time, // time
                pva );                    // PosVelAcc
  return this->push_without_velocity_change( tpva );
}


RetCode NonUniformRoundingSpline::push_dT( const double& interval_time,
                                           const double& position ) {
  double pre_time = 0.0;
  if( tpva_buffer_.size() > 0 ) {
    pre_time = tpva_buffer_.back().time;
  }
  return this->push( pre_time + interval_time, // time
                     position );               // position
}

RetCode NonUniformRoundingSpline::push_dT( const double& interval_time,
                                           const PosVelAcc& pva ) {
  double pre_time = 0.0;
  if( tpva_buffer_.size() > 0 ) {
    pre_time = tpva_buffer_.back().time;
  }
  TimePVA tpva( pre_time + interval_time, // time
                pva );                    // PosVelAcc
  return this->push( tpva );
}

RetCode NonUniformRoundingSpline::force_set_velocity( const std::size_t& index,
                                                      const double& velocity )
{
  TimePVA tpva = tpva_buffer_.get(index);
  tpva.P.vel   = velocity;
  return tpva_buffer_.set(index, tpva);
}


const TPVAQueue NonUniformRoundingSpline::tpva_queue() const {
  return tpva_buffer_;
}

const TimePVA NonUniformRoundingSpline::get( const std::size_t& index ) const {
  return tpva_buffer_.get(index);
}

const TimePVA NonUniformRoundingSpline::front() const {
  return tpva_buffer_.front();
}

const TimePVA NonUniformRoundingSpline::back() const {
  return tpva_buffer_.back();
}

const TimePVA NonUniformRoundingSpline::pop() {
  return tpva_buffer_.pop();
}

const TimePVA NonUniformRoundingSpline::pop_back() {
  return tpva_buffer_.pop_back();
}

RetCode NonUniformRoundingSpline::pop_delete() {
  return tpva_buffer_.pop_delete();
}

void NonUniformRoundingSpline::clear() {
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


