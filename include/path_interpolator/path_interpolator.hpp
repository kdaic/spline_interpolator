#ifndef INCLUDE_BASE_PATH_INTERPOLATOR_HPP_
#define INCLUDE_BASE_PATH_INTERPOLATOR_HPP_

#include <typeinfo>
#include <math.h>
#include <deque>

#include "plog/Log.h"
#include <plog/Appenders/ColorConsoleAppender.h>

#include "path_exception.hpp"

#define PRECISION 1.0e-15
namespace interp {

template<class T> bool g_nearEq(T a, T b) {
  if ( typeid(a) != typeid(int)
       && typeid(a) != typeid(float)
       && typeid(a) != typeid(double)
       && typeid(b) != typeid(int)
       && typeid(b) != typeid(float)
       && typeid(b) != typeid(double) ) {
    THROW( InvalidTypeArgument, "type must be numerical value");
  }
  if ( fabs(a - b) > PRECISION ) {
    return false;
  }
  return true;
}

template<class T> bool g_nearZero(T a) {
  if ( typeid(a) != typeid(int)
       && typeid(a) != typeid(float)
       && typeid(a) != typeid(double) ) {
    THROW( InvalidTypeArgument, "type must be numerical value");
  }
  if ( fabs(a) > PRECISION ) {
    return false;
  }
  return true;
}

/// Retrun Code
enum RetCode{
  PATH_SUCCESS=0,
  PATH_INVALID_INPUT_INDEX,
  PATH_INVALID_INPUT_TIME,
  PATH_INVALID_QUEUE,
  PATH_INVALID_QUEUE_SIZE,
  PATH_INVALID_ARGUMENT_VALUE_ZERO,
  PATH_QUEUE_SIZE_EMPTY,
  PATH_NOT_GENERATED,
  PATH_NOT_DEF_100PER_PATH,
  PATH_NOT_DEF_VEL_LIMIT,
  PATH_NOT_DEF_FUNCTION,
  PATH_NOT_RETURN
};

/// Struct data of Return Value
template<class T>
struct RetVal {
  /// Constructor
  RetVal(): retcode(PATH_NOT_RETURN) {};

  /// Constructor(data copy)
  RetVal(const RetCode& ret, const T& val=T()):
    retcode(ret), value(val) {};

  /// Copy operator
  RetVal operator=( const RetVal<T>& src ) {
    RetVal<T> dest(src.retcode, src.value);
    this->retcode = dest.retcode;
    this->value = dest.value;
    return *this;
  };

  /// Destructor
  ~RetVal() {};

  RetCode retcode;
  T value;
};


/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of Time,Position
struct TimePosition {
public:
  /// Constructor
  // TimePosition():time(0.0),position(0.0) {};
  TimePosition() {};

  /// Constructor(data copy)
  TimePosition( const double& _time, const double& _position=0.0 ):
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
  // TPV(): time(0.0),position(0.0),velocity(0.0) {};
  TPV(){};

  /// Constructor(data copy)
  TPV( const double& _time, const double& _position=0.0, const double& _velocity=0.0 ):
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
  virtual RetCode push( const T& newval );

  /// Pop T data from buffer queue(FIFO)
  /// @brief delete the pop data from queue_buffer_
  /// @return oldest T data
  RetVal<T> pop();

  /// Get TPV value at the index
  /// @return constant TPV at the index
  /// @exception If invalid index is accessed.
  const T get( const std::size_t& index ) const
    throw(InvalidIndexAccess);

  /// Set T value at the input-index
  /// @param[in] the index for setting T data
  /// @param[in] newval setting T new value
  /// @return setted T data
  /// - PATH_SUCCESS: No error
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const T newval );

  /// Clear all data of queue buffer
  void clear();

  /// Get queue size
  /// @return size of queue_buffer_
  const std::size_t size() const;

  /// Get dT at the input-index
  /// @param[in] index the index getting dT from queue
  /// @return dT at the input-index
  /// @exception If invalid index is accessed.
  const double dT( const std::size_t& index )
        throw(InvalidIndexAccess);

protected:
  /// calculate dT(t[index] - t[index-1]) internally at push() or set()
  /// @param[in] index the index calculating dT
  /// @return calculated dT
  virtual const double calc_dT( const std::size_t & index );

  /// The buffer instance
  std::deque<T> queue_buffer_;

  /// dT(t[index] - t[index-1]) queue calculated internally & automatically at the push()
  std::deque<double> dT_queue_;
}; // End of class TPVQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class Queue<TimePosition>;

// template
// class RetVal<TimePosition>;

/////////////////////////////////////////////////////////////////////////////////////////

/// TimePosition Queue buffer class
class TPQueue : public Queue<TimePosition> {
public:
  /// Constructor
  TPQueue();

  /// Destructor
  virtual ~TPQueue();

  /// Push TimePosition data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] TimePositon new target time position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  virtual RetCode push( const TimePosition& newTPval );

  /// Push TimePosition data into buffer queue(FIFO) (overload)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] position target position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const double& position );

  /// Set TimePosition value at the input-index (overload)
  /// @param[in] the index for setting TPV data
  /// @param[in] newTPval setting TPV value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const TimePosition& newTPval );

  /// calculate dT(t[index] - t[index-1]) internally at push() or set()
  /// @param[in] index the index calculating dT
  /// @return calculated dT
  virtual const double calc_dT( const std::size_t & index );
}; // End of class TPQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class Queue<TPV>;

// template
// class RetVal<TPV>;

/////////////////////////////////////////////////////////////////////////////////////////

/// TPV Queue buffer class
class TPVQueue : public Queue<TPV> {
public:
  /// Constructor
  /// @brief initial time is zero(0.0)
  TPVQueue();

  /// Destructor
  virtual ~TPVQueue();

  /// Push TPV data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] TgPV new target time position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  virtual RetCode push( const TPV& newTPVval );

  /// Push TPV data into buffer queue(FIFO) (overload)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] position target position
  /// @param[in] velocity target velocity
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const double& position,
                const double& velocity );

  /// Set TPV value at the input-index (overload)
  /// @param[in] the index for setting TPV data
  /// @param[in] newTPVval setting TPV value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const TPV& newTPVval );

  /// calculate dT(t[index] - t[index-1]) internally at push() or set()
  /// @param[in] index the index calculating dT
  /// @return calculated dT
  virtual const double calc_dT( const std::size_t & index );
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

 /// Get fiish-time
  /// @return
  /// - PATH_SUCCESS and tf
  /// - PATH_NOT_GENERATED and -1.0
  const RetVal<double> finish_time();

  /// Get limit velocity( if exists )
  /// @return limit velocity
  const RetVal<double> v_limit();

  /// Genrate a path from initial-finish point
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
  /// Path parameters are kept internally. \n
  /// If you only give start position & velocity and finish ones, \n
  /// (you give impossible finish time as finish time <= 0.0)
  ///
  /// ```
  /// (ts, xs, vs), (tf=?, xf, vf)
  /// ```
  ///
  /// interpolator calculates minmum interval time -> fiish time(tf). \n
  /// This means 100% mimum-time path in the limitation.
  virtual RetVal<double> generate_path( const double& xs, const double& xf,
                                        const double& vs=0.0, const double& vf=0.0,
                                        const double& ts=0.0, const double& tf=0.0 );

  /// Generate a path from Time, Position queue
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
  /// Interpolator sets start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs=0, as=0, js=0),
  /// (t1, x1, v1=?, a1=?, j1=?),
  /// (t2, x2, v2=?, a2=?, j2=?),
  ///  ...,
  /// (tf, xf, vf=0, af=0, jf=0)
  /// ```
  ///
  /// and interpolates intermediate (v1,a1,j1), (v2,a2,j2),.. automatically.
  virtual RetVal<double> generate_path( const TPQueue& tp_queue )=0;

  /// Generate a path from Time, Position(, Velocity) queue
  /// @param[in] Time, Position(, Velocity) queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  virtual RetVal<double> generate_path( const TPVQueue& tpv_queue )=0;

  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param t input time
  /// @return
  /// - PATH_SUCCESS & TPV at the input time
  /// - PATH_NOT_GENERATED and TPV is time=-1.0, position=0.0, velocity=0.0
  virtual RetVal<TPV> pop(const double& t)=0;

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
