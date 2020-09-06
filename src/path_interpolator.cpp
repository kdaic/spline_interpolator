#include "path_interpolator.hpp"

using namespace interp;


/////////////////////////////////////////////////////////////////////////////////////////

template
class RetVal<double>;

template
class RetVal<std::vector<double> >;

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
    throw InvalidIndexAccess( "source queue size is empty." );
  }
  this->clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

template<typename T>
RetCode TimeQueue<T>::push( const T& newval ) {
  queue_buffer_.push_back(newval);
  std::size_t last_index = queue_buffer_.size() - 1;
  // calculate dT if the queue left >= 1.
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
  }
  return PATH_SUCCESS;
}

template<typename T>
RetCode TimeQueue<T>::pop(T& output) {
  if( queue_buffer_.empty() ) {
    T retval();
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
const T TimeQueue<T>::get( const std::size_t& index ) const
  throw(InvalidIndexAccess) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    throw InvalidIndexAccess( "Queue size is empty." );
  }
  return queue_buffer_.at(index);
}

template<typename T>
RetCode TimeQueue<T>::set( const std::size_t& index, const T newval ) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }
  //
  queue_buffer_.at(index) = newval;
  //
  double dT;
  // front dT
  if( index >= 1 ){
    dT = calc_dT( index );
    dT_queue_.at( index - 1 ) = dT;
  }
  // back dT
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
const double TimeQueue<T>::dT( const std::size_t& index ) const
  throw(InvalidIndexAccess) {
  if( index < 0 || index > dT_queue_.size() - 1 ) {
    throw InvalidIndexAccess("Queue size is empty");
  }
  return dT_queue_[index];
}

template<typename T>
const double TimeQueue<T>::calc_dT( const std::size_t & index ) {
  if( queue_buffer_.size() < 2 ) {
    THROW( InvalidArgumentSize, "queue size must be >=2 for calculating dT list.");
  }
  if( index < 1 ) {
    THROW( InvalidArgumentValue, "index must be >=1 for calculating dT.");
  }
  /// This implement is not complete, please use or implement inherited class.
  return 0.0;
}


/////////////////////////////////////////////////////////////////////////////////////////

TPQueue::TPQueue() {
}

TPQueue::~TPQueue() {
}

RetCode TPQueue::push( const TimePosition& newTPval ) {
  if( queue_buffer_.size() > 0
      && newTPval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<TimePosition>::push( newTPval );
}


RetCode TPQueue::push( const double& time,
                       const double& position ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TimePosition newTPval(time, position);
  return TimeQueue<TimePosition>::push( newTPval );
}

RetCode TPQueue::set( const std::size_t& index,
                      const TimePosition& newTPval ) {
  if( ( index >= 1
        && newTPval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newTPval.time >= queue_buffer_[index+1].time ) ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<TimePosition>::set( index, newTPval );
}


const double TPQueue::calc_dT( const std::size_t & index ) {
  TimeQueue<TimePosition>::calc_dT( index );
  return queue_buffer_[index].time - queue_buffer_[index-1].time;
}

/////////////////////////////////////////////////////////////////////////////////////////


TPVQueue::TPVQueue() {
}

TPVQueue::~TPVQueue() {
}

RetCode TPVQueue::push( const TPV& newTPVval ) {
  if( queue_buffer_.size() > 0
      && newTPVval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<TPV>::push( newTPVval );
}

RetCode TPVQueue::push( const double& time,
                        const double& position,
                        const double& velocity ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TPV newTPVval(time, position, velocity);
  return TimeQueue<TPV>::push( newTPVval );
}

RetCode TPVQueue::set( const std::size_t& index,
                       const TPV& newTPVval ) {
  if( ( index >= 1
        && newTPVval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newTPVval.time >= queue_buffer_[index+1].time ) ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return TimeQueue<TPV>::set( index, newTPVval );
}

const double TPVQueue::calc_dT( const std::size_t & index ) {
  TimeQueue<TPV>::calc_dT( index );
  return queue_buffer_[index].time - queue_buffer_[index-1].time;
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
  out_dT = tpv_queue_.get( tpv_queue_.size() - 1 ).time - tpv_queue_.get(0).time;
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
  out_finish_time = tpv_queue_.get( tpv_queue_.size() - 1 ).time;
  return PATH_SUCCESS;
}

RetCode PathInterpolator::generate_path(
                            const double& xs, const double& xf,
                            const double& vs, const double& vf,
                            const double& dT ) {
  if( dT < 0.0  ) {
    return PATH_INVALID_INPUT_TIME;
  }

  if( g_nearZero(dT) ) {

    PosVel pvs(xs, vs);
    PosVel pvf(xf, vf);
    PVQueue pv_queue;
    pv_queue.push_back(pvs);
    pv_queue.push_back(pvf);

    return generate_path( pv_queue );

  } else {

    TPVQueue tpv_queue;
    tpv_queue.push( 0.0, xs, vs );
    tpv_queue.push( dT, xf, vf );

    return generate_path( tpv_queue );
  }

  return PATH_NOT_DEF_FUNCTION;
}
