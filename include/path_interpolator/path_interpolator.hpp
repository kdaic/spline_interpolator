#ifndef INCLUDE_BASE_PATH_INTERPOLATOR_HPP_
#define INCLUDE_BASE_PATH_INTERPOLATOR_HPP_
#include <cstdlib>

#include <string>
#include <sstream>
#include <iostream>

#include <math.h>
#include <deque>

#include <stdexcept>
#include <sysexits.h>

#include "plog/Log.h"
#include <plog/Appenders/ColorConsoleAppender.h>

#define PRECISION 1.0e-15
namespace interp {

/// Path Retrun Code
enum PathRetCode{
  PATH_SUCCESS=0,
  PATH_INVALID_INPUT_INDEX,
  PATH_INVALID_INPUT_TIME,
  PATH_NOT_GENERATED,
  PATH_NOT_DEF_100PER_PATH,
  PATH_NOT_DEF_VEL_LIMIT,
  PATH_NOT_DEF_FUNCTION,
  PATH_NOT_RETURN
};

template<class T> bool g_is_nearEq(T a, T b) {
  if ( fabs(a-b) > PRECISION ) {
    return false;
  }
  return true;
}

/// Struct data of Return Value
template<class T>
struct PathRetVal {
  /// Constructor
  PathRetVal(): retcode(PATH_NOT_RETURN), value(-1) {};

  /// Constructor(data copy)
  PathRetVal(const PathRetCode& ret, const T& val):
    retcode(ret), value(val) {};

  /// Copy operator
  PathRetVal operator=( const PathRetVal<T>& src ) {
    PathRetVal<T> dest(src.retcode, src.value);
    this->retcode = dest.retcode;
    this->value = dest.value;
    return *this;
  };

  /// Destructor
  ~PathRetVal() {};

  PathRetCode retcode;
  T value;
};

/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of Time,Position
struct TimePosition {
public:
  /// Constructor
  TimePosition(){};

  /// Constructor(data copy)
  TimePosition( const double& _time=0.0, const double& _position=0.0 ):
    time(_time), position(_position) {};

  /// Destructor
  ~TimePosition(){};

  /// Copy operator
  TimePosition operator=( const TimePosition& src ) {
    TimePosition dest(src.time, src.position);
    this->time = dest.time;
    this->position = dest.position;
    return *this;
  };
  /// time [sec]
  double time;
  /// postion [m, rad, ...etc.]
  double position;
}; // End of struct TimePosition


/// Struct data of Time,Position,Velocity
struct TPV {
public:
  /// Constructor
  TPV(){};

  /// Constructor(data copy)
  TPV( const double& _time=0.0, const double& _position=0.0, const double& _velocity=0.0 ):
    time(_time), position(_position), velocity(_velocity) {};

  /// Destructor
  ~TPV(){};

  /// Copy operator
  TPV operator=( const TPV& src ) {
    TPV dest(src.time, src.position, src.velocity);
    this->time = dest.time;
    this->position = dest.position;
    this->velocity = dest.velocity;
    return *this;
  };
  /// time [sec]
  double time;
  /// postion [m, rad, ...etc.]
  double position;
  /// velocity [m/s, rad/s, ...etc.]
  double velocity;

}; // End of struct TPV

/////////////////////////////////////////////////////////////////////////////////////////

/// Queue buffer base-class
template<class T>
class Queue {
public:
  /// Constructor
  Queue();

  /// Destructor
  virtual ~Queue();

  /// Copy operator
  /// @param[in] src Queue<T> source for copy
  /// @return copied instance of Queue<T>
  Queue<T> operator=( const Queue<T>& src );

  /// Push T data into buffer queue(FIFO)
  /// @param[in] newval T value source
  /// @brief push data T into the queue_buffer_
  /// @return
  /// - PATH_SUCCESS: no error
  virtual PathRetCode push( const T& newval );

  /// Pop T data from buffer queue(FIFO)
  /// @brief delete the pop data from queue_buffer_
  /// @return oldest T data
  T pop();

  /// Get TPV value at the index
  /// @return constant TPV
  const T get( const std::size_t index );

  /// Set T value at the input-index
  /// @param[in] the index for setting T data
  /// @param[in] newval setting T new value
  /// @return setted T data
  /// - PATH_SUCCESS: No error
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual PathRetCode set( const std::size_t index, const T newval );

  /// Clear all data of queue buffer
  void clear();

  /// Get queue size
  /// @return size of queue_buffer_
  const std::size_t size();

protected:
  /// The buffer instance
  std::deque<T> queue_buffer_;
}; // End of class TPVQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class Queue<TimePosition>;

/// TimePosition Queue buffer class
class TPQueue : public Queue<TimePosition> {
public:
  /// Constructor
  TPQueue();

  /// Destructor
  virtual ~TPQueue();

  /// Copy operator
  /// @param[in] src TPQueue source for copy
  /// @return copied instance of TPQueue
  TPQueue operator=( const TPQueue& src );

  /// Push TimePosition data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] TimePositon new target time position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  virtual PathRetCode push( const TimePosition& newTPval );

  /// Push TimePosition data into buffer queue(FIFO) (overload)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] position target position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  PathRetCode push( const double& time,
                    const double& position );

  /// Set TimePosition value at the input-index (overload)
  /// @param[in] the index for setting TPV data
  /// @param[in] tp_val setting TPV value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual PathRetCode set( const std::size_t index, const TimePosition& tp_val );

}; // End of class TPQueue

/////////////////////////////////////////////////////////////////////////////////////////

/// TPV Queue buffer class
class TPVQueue : public Queue<TPV> {
public:
  /// Constructor
  /// @brief initial time is zero(0.0)
  TPVQueue();

  /// Destructor
  virtual ~TPVQueue();

  /// Copy operator
  /// @param[in] src TPQueue source for copy
  /// @return copied instance of TPQueue
  TPVQueue operator=( const TPVQueue& src );

  /// Push TPV data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] TgPV new target time position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  virtual PathRetCode push( const TPV& newTPVval );

  /// Push TPV data into buffer queue(FIFO) (overload)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] position target position
  /// @param[in] velocity target velocity
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  PathRetCode push( const double& time,
                    const double& position,
                    const double& velocity );

  /// Set TPV value at the input-index (overload)
  /// @param[in] the index for setting TPV data
  /// @param[in] tpv_val setting TPV value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual PathRetCode set( const std::size_t index, const TPV& tpv_val );

}; // End of class TPVQueue


/////////////////////////////////////////////////////////////////////////////////////////

/// Base class of path interpolator
class PathInterpolator
{
public:

  /// Constructor
  PathInterpolator();

  /// Destructor
  virtual ~PathInterpolator();

  /// Genrate tragectory path from initial-finish point
  /// @param[in] xs start position
  /// @param[in] xf finish position
  /// @param[in] vs start velocity (default: 0.0)
  /// @param[in] vf finish velocity (default: 0.0)
  /// @param[in] ts start time (default: 0.0)
  /// @paran[in] tf finish time (default: 0.0)
  /// @brief calcurate tragectory parameter
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  /// @details
  /// If you only give start position & velocity and finish ones,
  /// (you give time zero as start time = 0.0, finish time = 0.0)
  ///
  /// ```
  /// (x_s, v_s), (x_f, v_f)
  /// ```
  ///
  /// interpolator calculates minmum interval time.
  /// This means 100% in the limitation.
  virtual PathRetVal<double> generate_path( const double& xs, const double& xf,
                                            const double& vs=0.0, const double& vf=0.0,
                                            const double& ts=0.0, const double& tf=0.0 )=0;

  /// Generate tragectory path from Time,Position queue
  /// @param[in] Time,Position queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  /// @details
  /// Input is TimePosition Queue like this.
  ///
  /// ```
  /// (ts, xs), (t1, x1), (t2, x2), ..., (tf, xf)
  /// ```
  ///
  /// Interpolator set start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs=0, as=0, js=0),
  /// (t1, x1, v1,   a1,   j1),
  /// (t2, x2, v2,   a2,   j2),
  ///  ...,
  /// (tf, xf, vf=0, a_f=0, j_f=0)
  /// ```
  ///
  /// and interpolate (v1,a1,j1), (v1,a1,j1),.. automatically.
  virtual PathRetVal<double> genrate_path( const TPQueue& tp_queue )=0;

  /// Generate tragectory path from Time,Position(,Velocity) queue
  /// @param[in] Time,Position(,Velocity) queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  virtual PathRetVal<double> genrate_path( const TPVQueue& tpv_queue )=0;

  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param t input time
  /// @return
  /// - PATH_SUCCESS & TPV at the input time
  /// - PATH_NOT_GENERATED and TPV is time=-1.0, position=0.0, velocity=0.0
  virtual PathRetVal<TPV> pop(const double& t)=0;

  /// Get fiish-time
  /// @return
  /// - PATH_SUCCESS and tf
  /// - PATH_NOT_GENERATED and -1.0
  const PathRetVal<double> finish_time();

  /// Get limit velocity( if exists )
  /// @return limit velocity
  const PathRetVal<double> v_limit();

  /// Get TPVQueue
  ///

protected:
  /// Set start and finish TPV
  void set_TPVsf( const double& ts, const double& tf,
                  const double& xs, const double& xf,
                  const double& vs=0.0, const double& vf=0.0 );

  /// flag if the path is generated. (default: false)
  bool is_path_generated_;

  /// flag if velocity limit (v_limit) is defined or not (default: false)
  bool is_v_limit_;

  /// start time [sec]
  double ts_;

  /// finish time [sec]
  double tf_;

  /// start position [m, rad, ...etc.]
  double xs_;

  /// finish position [m, rad, ...etc.]
  double xf_;

  /// start velocity [m/s, rad/s, ...etc.]
  double vs_;

  /// finish velocity [m/s, rad/s, ...etc.]
  double vf_;

  /// limit velocity [m/s, rad/s, ...etc.]
  /// (if defined)
  double v_limit_;

  /// TPQueue
  TPQueue tp_queue_;

  /// TPVQueue
  TPVQueue tpv_queue_;
}; // End of class PathInterpolator

} // End of namespace interp
#endif // INCLUDE_BASE_PATH_INTERPOLATOR_HPP_
