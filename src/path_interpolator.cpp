#include "path_interpolator.hpp"

using namespace interp;


/////////////////////////////////////////////////////////////////////////////////////////

template
class RetVal<double>;

template
class RetVal<std::vector<double> >;

/////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
Queue<T>::Queue() {
}

template<typename T>
Queue<T>::~Queue() {
}

template<typename T>
Queue<T> Queue<T>::operator=(const Queue<T>& src) {
  this->clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

template<typename T>
RetCode Queue<T>::push( const T& newval ) {
  queue_buffer_.push_back(newval);
  std::size_t last_index = queue_buffer_.size() - 1;
  if ( last_index >= 1 ) {
    double dT = calc_dT( last_index );
    dT_queue_.push_back(dT);
  }
  return PATH_SUCCESS;
}

template<typename T>
T Queue<T>::pop() {
  T front_val = queue_buffer_.front();
  queue_buffer_.pop_front();
  if( (queue_buffer_.size() >= 2) && (dT_queue_.size() > 1) ) {
    dT_queue_.pop_front();
  }
  return front_val;
}

template<typename T>
const T Queue<T>::get( const std::size_t& index ) const {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }
  return queue_buffer_.at(index);
}

template<typename T>
RetCode Queue<T>::set( const std::size_t& index, const T newval ) {
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
void Queue<T>::clear() {
  queue_buffer_.clear();
  dT_queue_.clear();
}

template<typename T>
const std::size_t Queue<T>::size() const {
  return queue_buffer_.size();
}

template<typename T>
const RetVal<double> Queue<T>::dT( const std::size_t& index ) {
  if( index < 0 || index > dT_queue_.size() - 1 ) {
    return RetVal<double>(PATH_INVALID_INPUT_INDEX, -1.0);
  }
  return RetVal<double>(PATH_SUCCESS, dT_queue_[index]);
}

template<typename T>
const double Queue<T>::calc_dT( const std::size_t & index ) {
  if( queue_buffer_.size() < 2 ) {
    THROW( InvalidArgumentSize, "queue size must be >=2 for calculating dT list.");
  }
  if( index < 1 ) {
    THROW( InvalidArgumentValue, "index must be >=1 for calculating dT.");
  }
  /// This implement is temporary, please use or implement inherited class.
  return 0.0;
}


/////////////////////////////////////////////////////////////////////////////////////////

TPQueue::TPQueue() {
}

TPQueue::~TPQueue() {
}

TPQueue TPQueue::operator=(const TPQueue& src) {
  queue_buffer_.clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

RetCode TPQueue::push( const TimePosition& newTPval ) {
  if( queue_buffer_.size() > 0
      && newTPval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TimePosition>::push( newTPval );
}


RetCode TPQueue::push( const double& time,
                       const double& position ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TimePosition newTPval(time, position);
  return Queue<TimePosition>::push( newTPval );
}

RetCode TPQueue::set( const std::size_t& index,
                      const TimePosition& newTPval ) {
  if( ( index >= 1
        && newTPval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newTPval.time >= queue_buffer_[index+1].time ) ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TimePosition>::set( index, newTPval );
}


const double TPQueue::calc_dT( const std::size_t & index ) {
  Queue<TimePosition>::calc_dT( index );
  return queue_buffer_[index].time - queue_buffer_[index-1].time;
}

/////////////////////////////////////////////////////////////////////////////////////////


TPVQueue::TPVQueue() {
}

TPVQueue::~TPVQueue() {
}

TPVQueue TPVQueue::operator=(const TPVQueue& src) {
  queue_buffer_.clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

RetCode TPVQueue::push( const TPV& newTPVval ) {
  if( queue_buffer_.size() > 0
      && newTPVval.time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TPV>::push( newTPVval );
}


RetCode TPVQueue::push( const double& time,
                        const double& position,
                        const double& velocity ) {
  if( queue_buffer_.size() > 0
      && time <= queue_buffer_.back().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TPV newTPVval(time, position, velocity);
  return Queue<TPV>::push( newTPVval );
}

RetCode TPVQueue::set( const std::size_t& index,
                       const TPV& newTPVval ) {
  if( ( index >= 1
        && newTPVval.time <= queue_buffer_[index-1].time )
      || ( index < queue_buffer_.size()-1
           && newTPVval.time >= queue_buffer_[index+1].time ) ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TPV>::set( index, newTPVval );
}

const double TPVQueue::calc_dT( const std::size_t & index ) {
  Queue<TPV>::calc_dT( index );
  return queue_buffer_[index].time - queue_buffer_[index-1].time;
}

/////////////////////////////////////////////////////////////////////////////////////////

PathInterpolator::PathInterpolator() :
  is_path_generated_(false), is_v_limit_(false) {
}

PathInterpolator::~PathInterpolator() {
}

const RetVal<double> PathInterpolator::finish_time() {
  if( !is_path_generated_ ) {
    return RetVal<double>(PATH_NOT_GENERATED, -1.0);
  }
  return RetVal<double>(PATH_SUCCESS, tf_);
}

const RetVal<double> PathInterpolator::v_limit() {
  if( !is_v_limit_ ) {
    return RetVal<double>(PATH_NOT_DEF_VEL_LIMIT, v_limit_);
  }
  return RetVal<double>(PATH_SUCCESS, v_limit_);
}

void PathInterpolator::set_TPVsf( const double& ts, const double& tf,
                                  const double& xs, const double& xf,
                                  const double& vs, const double& vf ) {
  ts_ = ts;
  tf_ = tf;
  xs_ = xs;
  xf_ = xf;
  vs_ = vs;
  vf_ = vf;
}

RetVal<double> PathInterpolator::generate_path(
                 const double& xs, const double& xf,
                 const double& vs, const double& vf,
                 const double& ts, const double& tf ) {
  if ( ts < 0.0 || ts >= tf ) {
    return RetVal<double>( PATH_INVALID_INPUT_TIME, -1.0 );
  }

  TPVQueue tpv_queue;
  tpv_queue.push( ts, xs, vs );
  tpv_queue.push( tf, xf, vf );

  set_TPVsf( ts, tf,  xs, xf,  ts, tf );

  return generate_path(tpv_queue);
}
