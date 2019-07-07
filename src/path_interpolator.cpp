#include "path_interpolator.hpp"

using namespace interp;

template<typename T>
Queue<T>::Queue() {
}

template<typename T>
Queue<T>::~Queue() {
}

template<typename T>
Queue<T> Queue<T>::operator=(const Queue<T>& src) {
  queue_buffer_.clear();
  queue_buffer_ = src.queue_buffer_;
  return *this;
}

template<typename T>
PathRetCode Queue<T>::push( const T& newval ) {
  queue_buffer_.push_back(newval);
  return PATH_SUCCESS;
}

template<typename T>
T Queue<T>::pop() {
  T front_val = queue_buffer_.front();
  queue_buffer_.pop_front();
  return front_val;
}

template<typename T>
const T Queue<T>::get( const std::size_t index ) {
  return queue_buffer_.at(index);
}

template<typename T>
PathRetCode Queue<T>::set( const std::size_t index, const T newval ) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }
  queue_buffer_.at(index) = newval;
  return PATH_SUCCESS;
}

template<typename T>
void Queue<T>::clear() {
  queue_buffer_.clear();
}

template<typename T>
const std::size_t Queue<T>::size() {
  return queue_buffer_.size();
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

PathRetCode TPQueue::push( const TimePosition& newTPval ) {
  if( queue_buffer_.size() > 0
      && newTPval.time < queue_buffer_.front().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TimePosition>::push( newTPval );
}


PathRetCode TPQueue::push( const double& time,
                           const double& position ) {
  if( queue_buffer_.size() > 0
      && time < queue_buffer_.front().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TimePosition newTPval(time, position);
  return Queue<TimePosition>::push( newTPval );
}

PathRetCode TPQueue::set( const std::size_t index,
                          const TimePosition& tp_val ) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }

  if( index > 0
      && tp_val.time < queue_buffer_[index].time ) {
    return PATH_INVALID_INPUT_TIME;
  }

  queue_buffer_.at(index) = tp_val;
  return PATH_SUCCESS;
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

PathRetCode TPVQueue::push( const TPV& newTPVval ) {
  if( queue_buffer_.size() > 0
      && newTPVval.time < queue_buffer_.front().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  return Queue<TPV>::push( newTPVval );
}


PathRetCode TPVQueue::push( const double& time,
                            const double& position,
                            const double& velocity ) {
  if( queue_buffer_.size() > 0
      && time < queue_buffer_.front().time ) {
    return PATH_INVALID_INPUT_TIME;
  }
  TPV newTPVval(time, position, velocity);
  return Queue<TPV>::push( newTPVval );
}

PathRetCode TPVQueue::set( const std::size_t index,
                           const TPV& tpv_val ) {
  if( index < 0 || index > queue_buffer_.size() -1 ) {
    return PATH_INVALID_INPUT_INDEX;
  }

  if( index > 0
      && tpv_val.time < queue_buffer_[index].time ) {
    return PATH_INVALID_INPUT_TIME;
  }

  queue_buffer_.at(index) = tpv_val;
  return PATH_SUCCESS;
}


/////////////////////////////////////////////////////////////////////////////////////////

template
class PathRetVal<double>;

PathInterpolator::PathInterpolator() :
  is_path_generated_(false), is_v_limit_(false) {
}

PathInterpolator::~PathInterpolator() {
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


const PathRetVal<double> PathInterpolator::finish_time() {
  if( !is_path_generated_ ) {
    return PathRetVal<double>(PATH_NOT_GENERATED, -1.0);
  }
  return PathRetVal<double>(PATH_SUCCESS, tf_);
}

const PathRetVal<double> PathInterpolator::v_limit() {
  if( !is_v_limit_ ) {
    return PathRetVal<double>(PATH_NOT_DEF_VEL_LIMIT, v_limit_);
  }
  return PathRetVal<double>(PATH_SUCCESS, v_limit_);
}
