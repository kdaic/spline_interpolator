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

/// judge nearly equal
/// @param[in] a
/// @param[in] b
/// @return true if a is nearly equal b within PRECISION, else returns false
template<class T> bool g_isNearlyEq(T a, T b) {
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

/// judge nearly zero
/// @param[in] a
/// @return true if a is nearly 0 within PRECISION, else returns false
template<class T> bool g_isNearlyZero(T a) {
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

/////////////////////////////////////////////////////////////////////////////////////////

/// Retrun Code
enum RetCode{
  PATH_SUCCESS=0,
  PATH_INVALID_INPUT_INDEX,
  PATH_INVALID_INPUT_TIME,
  PATH_INVALID_INPUT_INTERVAL_TIME_DT,
  PATH_INVALID_QUEUE,
  PATH_INVALID_QUEUE_SIZE,
  PATH_INVALID_ARGUMENT_VALUE_ZERO,
  PATH_INVALID_MATRIX_ARGUMENT_VALUE_ZERO,
  PATH_QUEUE_SIZE_EMPTY,
  PATH_NOT_GENERATED,
  PATH_NOT_DEF_100PER_PATH,
  PATH_NOT_DEF_VEL_LIMIT,
  PATH_NOT_DEF_FUNCTION,
  PATH_NOT_RETURN,
  PATH_TIME_IS_OUT_OF_RANGE
};

/// Struct data of Return Value
template<class T>
struct RetVal {
  /// Constructor
  RetVal(): code(PATH_NOT_RETURN) {};

  /// Constructor(data copy)
  RetVal(const RetCode& ret, const T& val=T()):
    code(ret), value(val) {};

  /// Copy operator
  RetVal operator=( const RetVal<T>& src ) {
    // copy
    RetVal<T> dest( src.code, src.value );
    this->code = dest.code;
    this->value = dest.value;
    return *this;
  };

  /// Destructor
  ~RetVal() {};

  /// return code
  RetCode code;
  /// return value
  T value;
};


/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of clock time, value
template<class T>
struct TimeVal {
  /// Constructor
  // TimeVal():time(0.0) {};
  TimeVal() {};

  /// Constructor(data copy)
  /// @param[in] _time  clock time
  /// @param[in] _value value
  TimeVal( const double& _time, const T& _value=0.0 ):
    time(_time), value(_value) {};

  /// Destructor
  ~TimeVal(){};

  /// Copy operator
  /// @param[in] src source which is TimeVal type
  TimeVal operator=( const TimeVal<T>& src ) {
    TimeVal<T> dest( src.time, src.value );
    this->time = dest.time;
    this->value = dest.value;
    return *this;
  };
  /// clock time [sec]
  double time;
  /// value [m, rad, ...etc.]
  T value;
}; // End of struct ClocktimeVal


/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of Position, Velocity, Acceleration
struct PosVelAcc {
public:
  /// Constructor
  // PosVelAcc(): position(0.0),velocity(0.0),acceleration(0.0) {};
  PosVelAcc(){};

  /// Constructor(data copy)
  /// @param[in] _position     initial position insert into pos
  /// @param[in] _velocity     initial velocity insert into vel
  /// @param[in] _acceleration initial acceleration insert into acc
  PosVelAcc( const double& _position,
             const double& _velocity=0.0,
             const double& _acceleration=0.0):
    pos(_position),
    vel(_velocity),
    acc(_acceleration) {};

  /// Destructor
  ~PosVelAcc(){};

  /// Copy operator
  /// @param[in] src source of copy which is type of PosVelAcc
  PosVelAcc operator=( const PosVelAcc& src ) {
    PosVelAcc dest( src.pos,
                    src.vel,
                    src.acc );
    this->pos = dest.pos;
    this->vel = dest.vel;
    this->acc = dest.acc;
    return *this;
  };
  /// position [m, rad, ...etc.]
  double pos;
  /// velocity [m/s, rad/s, ...etc.]
  double vel;
  /// acceleration [m/s^2, rad/s^2, ...etc.]
  double acc;

}; // End of struct TPV

/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of Time, Position, Velocity, Acceleration
struct TimePVA {
public:
  /// Constructor
  // TimePVA(): time(0.0),position(0.0),velocity(0.0), acceleration(0.0) {};
  TimePVA() :
    time(this->tpva_.time),
    pos(this->tpva_.value.pos),
    vel(this->tpva_.value.vel),
    acc(this->tpva_.value.acc) {};

  /// Constructor(data copy)
  /// @param[in] tpva source of copy which is type of TimePVA
  TimePVA( const TimePVA& tpva ) :
    time(this->tpva_.time),
    pos(this->tpva_.value.pos),
    vel(this->tpva_.value.vel),
    acc(this->tpva_.value.acc) {

    this->time = tpva.time;
    this->pos = tpva.pos;
    this->vel = tpva.vel;
    this->acc = tpva.acc;
  };

  /// Constructor(data copy)
  /// @param[in] tpva source of copy which is type of TimeVal<PosVelAcc>
  TimePVA( const TimeVal<PosVelAcc> tpva ) :
    tpva_(tpva),
    time(this->tpva_.time),
    pos(this->tpva_.value.pos),
    vel(this->tpva_.value.vel),
    acc(this->tpva_.value.acc) {};

  /// Constructor(data copy)
  /// @param[in] _time         initial time insert into time
  /// @param[in] _position     initial position insert into pos
  /// @param[in] _velocity     initial velocity insert into vel
  /// @param[in] _acceleration initial acceleration insert into acc
  TimePVA( const double& _time,
           const double& _position=0.0,
           const double& _velocity=0.0,
           const double& _acceleration=0.0 ) :
    time(this->tpva_.time),
    pos(this->tpva_.value.pos),
    vel(this->tpva_.value.vel),
    acc(this->tpva_.value.acc) {

    this->time = _time;
    this->pos = _position;
    this->vel = _velocity;
    this->acc = _acceleration;
  };

  /// Destructor
  ~TimePVA(){};

  /// Copy operator
  /// @param[in] src source of copy which is type of TimePVA
  TimePVA operator=( const TimePVA& src ) {
    TimePVA dest( src.time,
                  src.pos,
                  src.vel,
                  src.acc );
    this->time = dest.time;
    this->pos = dest.pos;
    this->vel = dest.vel;
    this->acc = dest.acc;
    return *this;
  };

  /// Copy operator
  /// @param[in] src source of copy which is type of TimeVal<PosVelAcc>
  TimePVA operator=( const TimeVal<PosVelAcc>& src ) {
    TimePVA dest( src.time,
                  src.value.pos,
                  src.value.vel,
                  src.value.acc);
    this->time = dest.time;
    this->pos = dest.pos;
    this->vel = dest.vel;
    this->acc = dest.acc;
    return *this;
  };


  /// source of TimeVal data
  TimeVal<PosVelAcc> tpva_;

  /// time [s]
  double& time;
  /// postion [m, rad, ...etc.]
  double& pos;
  /// velocity [m/s, rad/s, ...etc.]
  double& vel;
  /// acceleration [m/s^2, rad/s^2, ...etc.]
  double& acc;

}; // End of struct TPV

/////////////////////////////////////////////////////////////////////////////////////////

typedef std::vector<PosVelAcc> PVAList;

/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of the list of Time, Position, Velocity, Acceleration
struct TimePVAList {
public:
  /// Constructor
  // TimePVAList(): time(0.0),position(0.0),velocity(0.0), acceleration(0.0) {};
  TimePVAList() :
    time(this->tpva_list_.time),
    P(this->tpva_list_.value) {};

  /// Constructor
  /// @param[in] size the size of PVAList
  TimePVAList(std::size_t size) :
    time(this->tpva_list_.time),
    P(this->tpva_list_.value) {

    tpva_list_.value.resize(size);

  };

  /// Constructor(data copy)
  /// @param[in] src source of TimePVAList
  TimePVAList(const TimePVAList& src) :
    tpva_list_(src.tpva_list_),
    time(this->tpva_list_.time),
    P(this->tpva_list_.value) {};

  /// Constructor(data copy)
  /// @param[in] src source of TimeVal<PVAList> type
  TimePVAList( const TimeVal<PVAList> tpva_list) :
    tpva_list_(tpva_list),
    time(this->tpva_list_.time),
    P(this->tpva_list_.value) {};

  /// Constructor(data copy)
  /// @param[in] _time    clock time
  /// @param[in] pva_list the source of PVAList
  TimePVAList( const double& _time,
               const PVAList& pva_list ) :
    tpva_list_(_time,
               pva_list),
    time(this->tpva_list_.time),
    P(this->tpva_list_.value) {};

  /// Destructor
  ~TimePVAList(){};

  /// Copy operator
  /// @param[in] src source of copy which is type of TimePVAList
  TimePVAList operator=( const TimePVAList& src ) {
    this->time = src.time;
    this->P.resize( src.P.size() );
    for( std::size_t i=0; i<src.P.size(); i++ ) {
      this->P[i] = src.P[i];
    }
    return *this;
  };

  /// Copy operator
  /// @param[in] src source of copy which is type of TimeVal<PVAList>
  TimePVAList operator=( const TimeVal<PVAList>& src ) {
    this->time = src.time;
    this->P.resize( src.value.size() );
    for( std::size_t i=0; i<src.value.size(); i++ ) {
      this->P[i] = src.value[i];
    }
    return *this;
  };

  /// source of TimeVal<PVAList> data
  TimeVal<PVAList> tpva_list_;

  /// clock time [s] which is a reference of tpva_list_.time
  double& time;

  /// the list of postion, velocity, acceleration,
  /// which is a reference of tpva_list_.value
  PVAList& P;

}; // End of struct TimePVAList

/////////////////////////////////////////////////////////////////////////////////////////

/// Time Queue buffer base-class
template<class T>
class TimeQueue {
public:
  /// Constructor
  TimeQueue();

  /// Destructor
  virtual ~TimeQueue();

  /// Copy operator
  /// @param[in] src TimeQueue<T> source for copy
  /// @return copied instance of TimeQueue<T>
  TimeQueue<T> operator=( const TimeQueue<T>& src );

  /// Push TimeVal<T> data into buffer queue(FIFO)
  /// @param[in] newval TimVal<T> value source
  /// @brief push data TimVal<T> into the queue_buffer_
  /// @return
  /// - PATH_SUCCESS: no error
  virtual RetCode push( const TimeVal<T>& newval );

  /// Push clock time and value data into buffer queue(FIFO)
  /// @param[in] clocktime  clock time of TimVal<T> value source
  /// @param[in] value      value of TimVal<T> value source
  /// @brief push time and value into the queue_buffer_
  /// @return
  /// - PATH_SUCCESS: no error
  virtual RetCode push_on_clocktime( const double& clocktime,
                                     const T& value );

  /// Push interval time(dT) and value data into buffer queue(FIFO)
  /// @param[in] dT    interval time of TimVal<T> value source
  /// @param[in] value value of TimVal<T> value source
  /// @brief push time from intervaltime and value into the queue_buffer_
  /// @return
  /// - PATH_SUCCESS: no error
  virtual RetCode push_on_dT( const double& dT,
                              const T& value );


  /// Pop T data from buffer queue(FIFO)
  /// @brief delete the pop data from queue_buffer_
  /// @param[out] output oldest T data
  /// @return
  /// - PATH_QUEUE_SIZE_EMPTY: buffer size is not enough to pop.
  /// - PATH_SUCCESS: no error
  RetCode pop(TimeVal<T>& output);

  /// Get a value at the index
  /// @return constant a value at the index
  /// @exception If invalid index is accessed.
  const TimeVal<T> get( const std::size_t& index ) const
    throw(InvalidIndexAccess);

  /// Set T value at the input-index
  /// @param[in] the index for setting T data
  /// @param[in] newval setting T new value
  /// @return setted T data
  /// - PATH_SUCCESS: No error
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index,
                       const TimeVal<T> newval );

  /// Clear all data of queue buffer
  void clear();

  /// Get queue size
  /// @return size of queue_buffer_
  const std::size_t size() const;

  /// Get dT at the input-index
  /// @param[in] index the index getting dT from queue
  /// @return dT at the input-index
  /// @exception If invalid index is accessed.
  const double dT( const std::size_t& index ) const
        throw(InvalidIndexAccess);

protected:
  /// calculate the interval time(t[index] - t[index-1]) internally at push() or set()
  /// @param[in] index the index calculating intervaltime(dT)
  /// @return calculated dT
  virtual const double calc_dT( const std::size_t & index );

  /// The buffer instance
  std::deque<TimeVal<T> > queue_buffer_;

  /// The intervaltime(dT) (t[index] - t[index-1]) queue
  ///   calculated internally & automatically at the push()
  std::deque<double> dT_queue_;
}; // End of class TPVQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<double>;

/////////////////////////////////////////////////////////////////////////////////////////

/// TimePosition Queue buffer class
class TPQueue : public TimeQueue<double> {
public:
  /// Constructor
  TPQueue();

  /// Destructor
  virtual ~TPQueue();

}; // End of class TPQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<PosVelAcc>;

/////////////////////////////////////////////////////////////////////////////////////////

/// TPV Queue buffer class
class TPVAQueue : public TimeQueue<PosVelAcc> {
public:
  /// Constructor
  /// @brief initial time is zero(0.0)
  TPVAQueue();

  /// Destructor
  virtual ~TPVAQueue();

  /// Push TimePVA data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] TPV new target time position, velocity, acceleration
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const TimePVA& newval );

  /// Push TPV data into buffer queue(FIFO) (overload)
  /// @brief push TimePVA into the tpv_buffer_
  /// @param[in] time         target time
  /// @param[in] position     target position
  /// @param[in] velocity     target velocity
  /// @param[in] acceleration target acceleration
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const double& position,
                const double& velocity,
                const double& acceleration );

  /// Set TimePVA value at the input-index (overload)
  /// @param[in] index     the index for setting TPV data
  /// @param[in] newval setting TimePVA value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const TimePVA& newval );

}; // End of class TPVQueue

/////////////////////////////////////////////////////////////////////////////////////////

template
class TimeQueue<PVAList>;

/////////////////////////////////////////////////////////////////////////////////////////

/// TPVList Queue buffer class
class TPVAListQueue : public TimeQueue<PVAList> {
public:
  /// Constructor
  /// @brief initial time is zero(0.0)
  TPVAListQueue();

  /// Destructor
  virtual ~TPVAListQueue();

  /// Push TimePVAList data into buffer queue(FIFO)
  /// @brief push TimePVAList into the tpv_buffer_
  /// @param[in] newval TimePVAList new target time position
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const TimePVAList& newval );

  /// Push TimePVAList data into buffer queue(FIFO) (overload)
  /// @brief push time & PVAList into the tpv_buffer_
  /// @param[in] time     target time
  /// @param[in] pva_list target list of position, velocity, acceleration
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const PVAList& pva_list );

  /// Set TimePVAList value at the input-index (overload)
  /// @param[in] index     the index for setting TimePVA data
  /// @param[in] newval setting TimePVA value
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - PATH_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index,
                       const TimePVAList& newval );

}; // End of class TPVAListQueue


/////////////////////////////////////////////////////////////////////////////////////////

/// Position & Velocity Queue buffer class
typedef std::deque<PosVelAcc> PVAQueue;

/////////////////////////////////////////////////////////////////////////////////////////

/// Base class of path interpolator
class PathInterpolator
{
public:
  /// Constructor
  PathInterpolator();

  /// Destructor
  virtual ~PathInterpolator();

  /// Get total interval time
  /// @param[out] dT total interval time
  /// @return
  /// - PATH_SUCCESS
  /// - PATH_NOT_GENERATED and dT is -1.0
  const RetCode total_dT(double& out_dT);

  /// Get limit of velocity( if exists )
  /// @param[in] output limit of velocity
  /// @return limit velocity
  const RetCode v_limit(double& out_v_limit );

  /// Get finish time
  /// @param[out] finish_time time of path end
  /// @return finish time
  const RetCode finish_time(double& out_finish_time);

  /// Genrate a path from initial-finish point
  /// @param[in] xs           start position
  /// @param[in] xf           finish position
  /// @param[in] vs           start velocity (default: 0.0)
  /// @param[in] vf           finish velocity (default: 0.0)
  /// @param[in] as           start acceleration (default: 0.0)
  /// @param[in] af           finish acceleration (default: 0.0)
  /// @paran[in] dT           interval time (default: 0.0)
  /// @brief calcurate tragectory parameter
  /// @return
  /// - PATH_SUCCESS
  /// @details
  /// If you don't give interval time(dT=0.0),
  /// Minmum interval time dT are internally calculated automatically. \n
  /// This means 100% mimum-time path in the limitation.
  virtual RetCode generate_path( const double& xs, const double& xf,
                                 const double& vs=0.0, const double& vf=0.0,
                                 const double& as=0.0, const double& af=0.0,
                                 const double& dT=0.0 );

  /// Generate a path from Time, Position queue
  /// @param[in] Time,Position queue
  /// @param[in] vs start velocity (default: 0.0)
  /// @param[in] vf finish velocity (default: 0.0)
  /// @return
  /// - PATH_SUCCESS
  /// @details
  /// Input is TimePosition Queue like this.
  ///
  /// ```
  /// (ts, xs (, vs, as)), (t1, x1), (t2, x2), ..., (tf, xf(, vf, af))
  /// ```
  ///
  /// Interpolator sets start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs,   as, js=0),
  /// (t1, x1, v1=?, a1=?, j1=?),
  /// (t2, x2, v2=?, a2=?, j2=?),
  ///  ...,
  /// (tf, xf, vf,   af=0, jf=0)
  /// ```
  ///
  /// and interpolates intermediate (v1,a1,j1), (v2,a2,j2),.. automatically.
  virtual RetCode generate_path( const TPQueue& tp_queue,
                                 const double vs=0.0, const double vf=0.0,
                                 const double as=0.0, const double af=0.0)=0;

  /// Generate a path from Time, Position(, Velocity) queue
  /// @param[in] Time, Position(, Velocity, Acceleration) queue
  /// @return
  /// - PATH_SUCCESS
  virtual RetCode generate_path( const TPVAQueue& tpva_queue )=0;

  /// Generate a path from Position(, Velocity) queue
  /// @param[in] pv_queue Position, Velocity queue
  /// @return
  /// - PATH_SUCCESS and total interval time (tf - ts)
  /// you only give start position & velocity & acceleration and don't give time. \n
  ///
  /// ```
  /// (xs, vs, as), (x1, v1, a1), (x2, v2, a2),..., (xf, vf, af)
  /// ```
  ///
  /// Minmum interval time -> ts=0, t1, t2,..., tf are internally calculated automatically. \n
  /// This means 100% mimum-time path in the limitation.
  virtual RetCode generate_path( const PVAQueue& pva_queue )=0;

  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param[in] t input time
  /// @param[out] output TPV at the input time
  /// @return
  /// - PATH_SUCCESS: no error
  /// - PATH_NOT_GENERATED and TPV: is time=-1.0, position=0.0, velocity=0.0
  /// - PATH_TIME_IS_OUT_OF_RANGE: time is not within the range of generated path
  virtual RetCode pop(const double& t, TimePVA& output)=0;

protected:
  /// flag if the path is generated. (default: false)
  bool is_path_generated_;

  /// flag if velocity limit (v_limit) is defined or not (default: false)
  bool is_v_limit_;

  /// limit velocity [m/s, rad/s, ...etc.]
  /// (if defined)
  double v_limit_;

  /// TPVQueue
  TPVAQueue tpva_queue_;
}; // End of class PathInterpolator

} // End of namespace interp
#endif // INCLUDE_BASE_PATH_INTERPOLATOR_HPP_
