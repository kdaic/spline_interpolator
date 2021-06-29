#include "spline_data.hpp"

using namespace interp;

/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<double>;

/////////////////////////////////////////////////////////////////////////////////////////


TPQueue::TPQueue() {
}

TPQueue::~TPQueue() {
}

RetCode TPQueue::dump( std::string& dest_queue_dump ) {
  dest_queue_dump.clear();
  std::stringstream ss;
  for( std::size_t i=0; i<queue_buffer_.size(); i++ ) {
    ss << queue_buffer_[i].time << ", "
       << queue_buffer_[i].value << std::endl;;
  }
  dest_queue_dump = ss.str();
  return SPLINE_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<PosVelAcc>;

/////////////////////////////////////////////////////////////////////////////////////////


TPVAQueue::TPVAQueue() {
}

TPVAQueue::~TPVAQueue() {
}

RetCode TPVAQueue::push( const TimePVA& newval ) {
  if( queue_buffer_.size() > 0
      && newval.time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  return TimeQueue<PosVelAcc>::push( newval );
}

RetCode TPVAQueue::push( const double& time,
                         const PosVelAcc& pva ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  TimePVA newval( time,
                  PosVelAcc(pva.pos, pva.vel, pva.acc) );
  return TimeQueue<PosVelAcc>::push( newval );
}

RetCode TPVAQueue::push( const double& time,
                         const double& position,
                         const double& velocity,
                         const double& acceleration ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  TimePVA newval( time,
                  PosVelAcc(position, velocity, acceleration) );
  return TimeQueue<PosVelAcc>::push( newval );
}

RetCode TPVAQueue::set( const std::size_t& index,
                        const TimePVA& newval ) {
  if( ( index >= 1
        && newval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newval.time >= queue_buffer_[index+1].time ) ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  return TimeQueue<PosVelAcc>::set( index, newval );
}

RetCode TPVAQueue::dump( std::string& dest_queue_dump ) {
  dest_queue_dump.clear();
  std::stringstream ss;
  for( std::size_t i=0; i<queue_buffer_.size(); i++ ) {
    ss << queue_buffer_[i].time << ", ["
       << queue_buffer_[i].P.pos << ", "
       << queue_buffer_[i].P.vel << ", "
       << queue_buffer_[i].P.acc << "]" << std::endl;
  }
  dest_queue_dump = ss.str();
  return SPLINE_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<PVAList>;

/////////////////////////////////////////////////////////////////////////////////////////

TPVAListQueue::TPVAListQueue() {
}

TPVAListQueue::~TPVAListQueue() {
}

RetCode TPVAListQueue::push( const TimePVAList& newval ) {
  if( queue_buffer_.size() > 0
      && newval.time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  return TimeQueue<PVAList>::push( newval );
}

RetCode TPVAListQueue::push( const double& time,
                             const PVAList& pva_list ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  TimePVAList newval(time, pva_list);
  return TimeQueue<PVAList>::push( newval );
}

RetCode TPVAListQueue::set( const std::size_t& index,
                            const TimePVAList& newval ) {
  if( ( index >= 1
        && newval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newval.time >= queue_buffer_[index+1].time ) ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  return TimeQueue<PVAList>::set( index, newval );
}

RetCode TPVAListQueue::dump( std::string& dest_queue_dump ) {
  dest_queue_dump.clear();
  std::stringstream ss;
  for( std::size_t i=0; i<queue_buffer_.size(); i++ ) {
    ss << queue_buffer_[i].time << ", [";
    std::size_t tpvalist_size = queue_buffer_[i].value.size();
    for( std::size_t j=0; j<tpvalist_size; j++ ) {
      ss << "["
         << queue_buffer_[i].P[j].pos << ", "
         << queue_buffer_[i].P[j].vel << ", "
         << queue_buffer_[i].P[j].acc << "]";
      if( j < tpvalist_size-1 ) { ss << ", "; }
    }
    ss << "]" << std::endl;
  }
  dest_queue_dump = ss.str();
  return SPLINE_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////

TrapezoidConfig::TrapezoidConfig(
                 const double& _a_limit,
                 const double& _d_limit,
                 const double& _v_limit,
                 const double& _asr,
                 const double& _dsr,
                 const double& _ratio_acc_dec ) :
  a_limit(_a_limit), d_limit(_d_limit),
  v_limit(_v_limit),
  asr(_asr), dsr(_dsr),
  ratio_acc_dec(_ratio_acc_dec) {
}

TrapezoidConfig::TrapezoidConfig(
                 const TrapezoidConfig& src ) :
  a_limit(src.a_limit), d_limit(src.d_limit),
  v_limit(src.v_limit),
  asr(src.asr), dsr(src.dsr),
  ratio_acc_dec(src.ratio_acc_dec) {
}

TrapezoidConfig& TrapezoidConfig::operator=(
                                 const TrapezoidConfig& src ) {
  TrapezoidConfig dest( src );
  this->a_limit = dest.a_limit;
  this->d_limit = dest.d_limit;
  this->v_limit = dest.v_limit;
  this->asr = dest.asr;
  this->dsr = dest.dsr;
  this->ratio_acc_dec = dest.ratio_acc_dec;
  return *this;
}

/////////////////////////////////////////////////////////////////////////////////////////
