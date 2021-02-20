#include "path_interpolator.hpp"

using namespace interp;


/////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
TimeQueue<T>::TimeQueue() {
}

template<typename T>
TimeQueue<T>::~TimeQueue() {
}

template<typename T>
TimeQueue<T> TimeQueue<T>::operator=(const TimeQueue<T>& src) {
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
    return PATH_INVALID_INPUT_TIME;
  }

  queue_buffer_.push_back(newval);

  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate intervaltime(dT) if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
  }
  return PATH_SUCCESS;
}


template<typename T>
RetCode TimeQueue<T>::push_on_clocktime(
                        const double& clocktime,
                        const T& value ) {
  if( queue_buffer_.size() > 0
      && clocktime <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }

  TimeVal<T> newval(clocktime, value);
  queue_buffer_.push_back(newval);

  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate intervaltime(dT) if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
  }
  return PATH_SUCCESS;
}


template<typename T>
RetCode TimeQueue<T>::pop(TimeVal<T>& output) {
  if( queue_buffer_.empty() ) {
    return PATH_QUEUE_SIZE_EMPTY;
  }
  output = queue_buffer_.front();
  queue_buffer_.pop_front();
  if( (queue_buffer_.size() >= 2) && (dT_queue_.size() > 1) ) {
    dT_queue_.pop_front();
  }
  return PATH_SUCCESS;
}

template<typename T>
RetCode TimeQueue<T>::pop_delete() {
  if( queue_buffer_.empty() ) {
    return PATH_QUEUE_SIZE_EMPTY;
  }
  queue_buffer_.pop_front();
  if( (queue_buffer_.size() >= 2) && (dT_queue_.size() > 1) ) {
    dT_queue_.pop_front();
  }
  return PATH_SUCCESS;
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
  }
  return PATH_SUCCESS;
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
  if( ( index >= 1
        && newval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newval.time >= queue_buffer_[index+1].time ) ) {
    return PATH_INVALID_INPUT_TIME;
  }

  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }
  //
  queue_buffer_.at(index) = newval;
  //
  double dT;
  // front intervaltime(dT)
  if( index >= 1 ){
    dT = calc_dT( index );
    dT_queue_.at( index - 1 ) = dT;
  }
  // back intervaltime(dT)
  if( queue_buffer_.size() > index + 1 ) {
    dT = calc_dT( index + 1 );
    dT_queue_.at( index ) = dT;
  }

  return PATH_SUCCESS;
}

template<typename T>
void TimeQueue<T>::clear() {
  queue_buffer_.clear();
  dT_queue_.clear();
}

template<typename T>
const std::size_t TimeQueue<T>::size() const {
  return queue_buffer_.size();
}

template<typename T>
RetCode TimeQueue<T>::dump( std::string& queue_dump ) {
  return PATH_SUCCESS;
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


TPQueue::TPQueue() {
}

TPQueue::~TPQueue() {
}

RetCode TPQueue::dump( std::string& queue_dump ) {
  queue_dump.clear();
  std::stringstream ss;
  for( std::size_t i=0; i<queue_buffer_.size(); i++ ) {
    ss << queue_buffer_[i].time << ", "
       << queue_buffer_[i].value << std::endl;;
  }
  queue_dump = ss.str();
  return PATH_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////


TPVAQueue::TPVAQueue() {
}

TPVAQueue::~TPVAQueue() {
}

RetCode TPVAQueue::push( const TimePVA& newval ) {
  if( queue_buffer_.size() > 0
      && newval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<PosVelAcc>::push( newval );
}

RetCode TPVAQueue::push( const double& time,
                         const PosVelAcc& pva ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
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
    return PATH_INVALID_INPUT_TIME;
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
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<PosVelAcc>::set( index, newval );
}

RetCode TPVAQueue::dump( std::string& queue_dump ) {
  queue_dump.clear();
  std::stringstream ss;
  for( std::size_t i=0; i<queue_buffer_.size(); i++ ) {
    ss << queue_buffer_[i].time << ", ["
       << queue_buffer_[i].P.pos << ", "
       << queue_buffer_[i].P.vel << ", "
       << queue_buffer_[i].P.acc << "]" << std::endl;
  }
  queue_dump = ss.str();
  return PATH_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////

TPVAListQueue::TPVAListQueue() {
}

TPVAListQueue::~TPVAListQueue() {
}

RetCode TPVAListQueue::push( const TimePVAList& newval ) {
  if( queue_buffer_.size() > 0
      && newval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<PVAList>::push( newval );
}

RetCode TPVAListQueue::push( const double& time,
                             const PVAList& pva_list ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
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
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<PVAList>::set( index, newval );
}

RetCode TPVAListQueue::dump( std::string& queue_dump ) {
  queue_dump.clear();
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
  queue_dump = ss.str();
  return PATH_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////

PathInterpolator::PathInterpolator() :
  is_path_generated_(false), is_v_limit_(false) {
}

PathInterpolator::~PathInterpolator() {
}

const RetCode PathInterpolator::total_dT(double& out_dT) {
  if( !is_path_generated_ ) {
    out_dT = -1.0;
    return PATH_NOT_GENERATED;
  }
  out_dT = tpva_queue_.get( tpva_queue_.size() - 1 ).time - tpva_queue_.get(0).time;
  return PATH_SUCCESS;
}

const RetCode PathInterpolator::v_limit(double& out_v_limit) {
  if( !is_v_limit_ ) {
    out_v_limit = v_limit_;
    return PATH_NOT_DEF_VEL_LIMIT;
  }
  out_v_limit = v_limit_;
  return PATH_SUCCESS;
}

const RetCode PathInterpolator::finish_time(double& out_finish_time) {
  if( !is_path_generated_ ) {
    out_finish_time = -1.0;
    return PATH_NOT_GENERATED;
  }
  out_finish_time = tpva_queue_.get( tpva_queue_.size() - 1 ).time;
  return PATH_SUCCESS;
}

RetCode PathInterpolator::generate_path(
                            const double& xs, const double& xf,
                            const double& vs, const double& vf,
                            const double& as, const double& af,
                            const double& dT ) {
  if( dT < 0.0 ) {
    return PATH_INVALID_INPUT_TIME;
  }

  PosVelAcc pvas(xs, vs, as);
  PosVelAcc pvaf(xf, vf, af);
  if( g_isNearlyZero(dT) ) {

    PVAQueue pva_queue;
    pva_queue.push_back(pvas);
    pva_queue.push_back(pvaf);

    return generate_path( pva_queue );

  } else {

    TPVAQueue tpva_queue;
    tpva_queue.push_on_clocktime( 0.0, pvas );
    tpva_queue.push_on_dT( dT, pvaf );

    return generate_path( tpva_queue );
  }

  return PATH_NOT_DEF_FUNCTION;
}
