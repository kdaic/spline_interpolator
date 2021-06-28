#include "spline_data.hpp"

using namespace interp;

/////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
TimeQueue<T>::TimeQueue() :
  total_dT_(0.0) {
}

template<typename T>
TimeQueue<T>::~TimeQueue() {
}

template<typename T>
TimeQueue<T>& TimeQueue<T>::operator=(const TimeQueue<T>& src) {
  if( src.size() < 1 ) {
    THROW( InvalidIndexAccess, "source queue size is empty." );
  }
  this->clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

template<typename T>
RetCode TimeQueue<T>::push( const TimeVal<T>& newval ) {
  if( queue_buffer_.size() > 0
      && newval.time <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }

  queue_buffer_.push_back(newval);

  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate intervaltime(dT) if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
    total_dT_ += dT;
  }
  return SPLINE_SUCCESS;
}


template<typename T>
RetCode TimeQueue<T>::push_on_clocktime(
                        const double& clocktime,
                        const T& value ) {
  if( queue_buffer_.size() > 0
      && clocktime <= queue_buffer_.back().time ) {
    return SPLINE_INVALID_INPUT_TIME;
  }

  TimeVal<T> newval(clocktime, value);
  queue_buffer_.push_back(newval);

  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate intervaltime(dT) if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
    total_dT_ += dT;
  }
  return SPLINE_SUCCESS;
}


template<typename T>
const TimeVal<T> TimeQueue<T>::pop() {
  if( queue_buffer_.empty() ) {
    THROW( QueueSizeEmpty, "the size of time queue is empty" );
  }
  TimeVal<T> output = queue_buffer_.front();
  queue_buffer_.pop_front();
  if( (queue_buffer_.size() >= 2) && (dT_queue_.size() > 1) ) {
    dT_queue_.pop_front();
    total_dT_ -= output.time;
  }
  return output;
}

template<typename T>
RetCode TimeQueue<T>::pop_delete() {
  if( queue_buffer_.empty() ) {
    THROW( QueueSizeEmpty, "the size of time queue is empty" );
  }
  double pop_time = queue_buffer_.front().time;
  queue_buffer_.pop_front();
  if( (queue_buffer_.size() >= 2) && (dT_queue_.size() > 1) ) {
    dT_queue_.pop_front();
    total_dT_ -= pop_time;
  }
  return SPLINE_SUCCESS;
}


template<typename T>
RetCode TimeQueue<T>::push_on_dT(
                        const double& dT,
                        const T& value ) {
  double clocktime = 0.0;
  if( queue_buffer_.size() > 0 ) {
    clocktime = queue_buffer_.back().time + dT;
  } else {
    clocktime = dT;
  }

  TimeVal<T> newval(clocktime, value);
  queue_buffer_.push_back(newval);

  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate intervaltime(dT) if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
    total_dT_ += dT;
  }
  return SPLINE_SUCCESS;
}

template<typename T>
const TimeVal<T> TimeQueue<T>::get( const std::size_t& index ) const
  throw(InvalidIndexAccess) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    THROW( InvalidIndexAccess, "Queue index is invalid." );
  }
  return queue_buffer_.at(index);
}

template<typename T>
const TimeVal<T> TimeQueue<T>::front() const
  throw(InvalidIndexAccess) {
  if( queue_buffer_.empty() ) {
    THROW( InvalidIndexAccess, "Queue size is empty." );
  }
  return queue_buffer_.front();
}

template<typename T>
const TimeVal<T> TimeQueue<T>::back() const
  throw(InvalidIndexAccess) {
  if( queue_buffer_.empty() ) {
    THROW( InvalidIndexAccess, "Queue size is empty." );
  }
  return queue_buffer_.back();
}


template<typename T>
RetCode TimeQueue<T>::set( const std::size_t& index,
                           const TimeVal<T> newval ) {

  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return SPLINE_INVALID_INPUT_INDEX;
  }

  if( ( index >= 1
        && newval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newval.time >= queue_buffer_[index+1].time ) ) {
    return SPLINE_INVALID_INPUT_TIME;
  }
  //
  queue_buffer_.at(index) = newval;
  //
  double dT;
  // front intervaltime(dT)
  if( index >= 1 ){
    dT = calc_dT( index );
    total_dT_ -= dT_queue_.at( index - 1 );
    dT_queue_.at( index - 1 ) = dT;
    total_dT_ += dT;
  }
  // back intervaltime(dT)
  if( queue_buffer_.size() > index + 1 ) {
    dT = calc_dT( index + 1 );
    total_dT_ -= dT_queue_.at( index );
    dT_queue_.at( index ) = dT;
    total_dT_ += dT;
  }

  return SPLINE_SUCCESS;
}

template<typename T>
void TimeQueue<T>::clear() {
  queue_buffer_.clear();
  dT_queue_.clear();
  total_dT_ = 0.0;
}

template<typename T>
const std::size_t TimeQueue<T>::size() const {
  return queue_buffer_.size();
}

template<typename T>
RetCode TimeQueue<T>::dump( const std::string& queue_dump ) {
  return SPLINE_SUCCESS;
}

template<typename T>
const double TimeQueue<T>::dT( const std::size_t& index ) const
  throw(InvalidIndexAccess) {
  if( index < 0 || index > dT_queue_.size() - 1 ) {
    THROW( InvalidIndexAccess, "Queue size is empty" );
  }
  return dT_queue_[index];
}

template<typename T>
const double TimeQueue<T>::total_dT() const {
  return total_dT_;
}

template<typename T>
const double TimeQueue<T>::calc_dT( const std::size_t & index ) {
  if( queue_buffer_.size() < 2 ) {
    THROW( InvalidArgumentSize, "queue size must be >=2 for calculating intervaltime(dT) list.");
  }
  if( index < 1 ) {
    THROW( InvalidArgumentValue, "index must be >=1 for calculating intervaltime(dT).");
  }
  return queue_buffer_[index].time - queue_buffer_[index-1].time;
}


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
