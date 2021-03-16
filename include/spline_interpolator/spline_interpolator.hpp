#ifndef INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
#define INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_

#include <typeinfo>
#include <math.h>
#include <vector>
#include <deque>
#include <limits> // for PRECISION ( using epsilon )

#include "spline_exception.hpp"

const double PRECISION = std::numeric_limits<double>::epsilon();

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
  SPLINE_SUCCESS=0,
  SPLINE_INVALID_INPUT_INDEX,
  SPLINE_INVALID_INPUT_TIME,
  SPLINE_INVALID_INPUT_INTERVAL_TIME_DT,
  SPLINE_INVALID_QUEUE,
  SPLINE_INVALID_QUEUE_SIZE,
  SPLINE_INVALID_ARGUMENT_VALUE_ZERO,
  SPLINE_INVALID_MATRIX_ARGUMENT_VALUE_ZERO,
  SPLINE_NOT_DEF_100PER_PATH,
  SPLINE_NOT_DEF_FUNCTION,
  SPLINE_NOT_RETURN
};

/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of clock time, value
template<class T>
struct TimeVal {
  /// Constructor
  // TimeVal():time(0.0) {};
  TimeVal() :
    time(0.0), P(this->value) {};

  /// Copy Constructor
  /// @param[in] src source of TimeVal
  TimeVal( const TimeVal& src ) :
    time(src.time), value(src.value), P(value) {};

  /// Constructor(data copy)
  /// @param[in] _time  clock time
  /// @param[in] _value value
  TimeVal( const double& _time, const T& _value=T() ):
    time(_time), value(_value), P(value) {};

  /// Destructor
  ~TimeVal(){};

  /// Copy operator
  /// @param[in] src source which is TimeVal type
  TimeVal operator=( const TimeVal<T>& src ) {
    // against self copy
    TimeVal<T> dest( src.time, src.value );
    this->time  = dest.time;
    this->value = dest.value;
    return *this;
  };
  /// clock time [sec]
  double time;
  /// value [m, rad, ...etc.]
  T value;
  /// the reference of value
  T& P;
}; // End of struct TimeVal

/////////////////////////////////////////////////////////////////////////////////////////

template
struct TimeVal<double>;

/////////////////////////////////////////////////////////////////////////////////////////

// /// type definition of Time-Position struct data derived from TimeVal<T>
typedef TimeVal<double> TimePosition;

/////////////////////////////////////////////////////////////////////////////////////////

/// Struct data of Position, Velocity, Acceleration
struct PosVelAcc {
public:
  /// Constructor
  // PosVelAcc(): position(0.0),velocity(0.0),acceleration(0.0) {};
  PosVelAcc() :
    pos(0.0),
    vel(0.0),
    acc(0.0) {};

  /// Copy Constructor
  /// @param[in] src source PosVelAcc
  PosVelAcc( const PosVelAcc& src ) :
    pos(src.pos),
    vel(src.vel),
    acc(src.acc) {};

  /// Constructor(data copy)
  /// @param[in] _position     initial position insert into pos
  /// @param[in] _velocity     initial velocity insert into vel
  /// @param[in] _acceleration initial acceleration insert into acc
  PosVelAcc( const double& _position,
             const double& _velocity=0.0,
             const double& _acceleration=0.0) :
    pos(_position),
    vel(_velocity),
    acc(_acceleration) {};

  /// Destructor
  ~PosVelAcc(){};

  /// Copy operator
  /// @param[in] src source of copy which is type of PosVelAcc
  PosVelAcc operator=( const PosVelAcc& src ) {
    // against self copy
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

template
struct TimeVal<PosVelAcc>;

/////////////////////////////////////////////////////////////////////////////////////////

typedef TimeVal<PosVelAcc> TimePVA;

/////////////////////////////////////////////////////////////////////////////////////////

struct PVAList {
  /// Constructor
  PVAList(){};

  /// Destructor
  ~PVAList(){};

  /// Constructor(data copy)
  /// @param[in] src source of PVAList type
  PVAList( const PVAList& src ) {
    this->pvalist.resize( 0 );
    std::copy( src.pvalist.begin(), src.pvalist.begin() + src.pvalist.size(),
               std::back_inserter(this->pvalist) );
  };

  /// Constructor(data copy)
  /// @param[in] src source of vector<PosVelAcc> type
  PVAList( const std::vector<PosVelAcc>& src ) {
    this->pvalist.resize( 0 );
    std::copy( src.begin(), src.begin() + src.size(),
               std::back_inserter(this->pvalist) );
  };

  /// Constructor(data copy)
  /// @param[in] _pos  the source of position
  /// @param[in] _vel  the source of velocity
  /// @param[in] _acc  the source of acceleration
  PVAList( const std::vector<double>& _pos,
           const std::vector<double>& _vel,
           const std::vector<double>& _acc ) {
    if( ( _pos.size() != _vel.size() ) ||
        ( _pos.size() != _acc.size() ) ) {
      THROW( InvalidArgumentSize,
             "all argument sizes must be matched." );
    }
    this->pvalist.resize( _pos.size() );
    for( std::size_t i=0; i<_pos.size(); i++ ) {
      this->pvalist[i].pos = _pos[i];
      this->pvalist[i].vel = _vel[i];
      this->pvalist[i].acc = _acc[i];
    }
  };

  /// Copy operator
  /// @param[in] src source of copy which is type of TimePVAList
  /// @return *this
  PVAList operator=( const PVAList& src ) {
    // for self copy
    PVAList dest( src );
    this->pvalist.clear();
    this->pvalist.resize(0);
    std::copy( dest.pvalist.begin(), dest.pvalist.begin() + dest.pvalist.size(),
               std::back_inserter(this->pvalist) );
    return *this;
  };

  /// Copy operator
  /// @param[in] src the source of copy which is type of TimePVAList
  /// @return *this
  PVAList operator=( const std::vector<PosVelAcc>& src ) {
    // for self copy
    std::vector<PosVelAcc> dest = src;
    this->pvalist.clear();
    this->pvalist.resize(0);
    std::copy( dest.begin(), dest.begin() + dest.size(),
               std::back_inserter(this->pvalist) );
    return *this;
  };

  /// @param[in] index the index of list
  /// @return the reference of PosVelAcc[index]
  PosVelAcc& operator[]( std::size_t index ) {
    return this->pvalist[index];
  };

  /// @param[in] index the index of list
  /// @return the reference of PosVelAcc[index].pos
  double& pos( std::size_t index ) {
    return this->pvalist[index].pos;
  }

  /// @param[in] index index of list
  /// @return the reference of PosVelAcc[index].vel
  double& vel( std::size_t index ) {
    return this->pvalist[index].vel;
  }

  /// @param[in] index the index of list
  /// @return the reference of PosVelAcc[index].acc
  double& acc( std::size_t index ) {
    return this->pvalist[index].acc;
  }

  /// returns the list size of PVAList
  /// @return the list size of PVAList
  std::size_t size() {
    return this->pvalist.size();
  };

  /// resize resize of pvalist
  /// @param[in] size the size of resize
  void resize( std::size_t size ) {
    this->pvalist.resize(size);
  }

  /// clear the pvalist
  void clear() {
    this->pvalist.clear();
  }

  /// pushing back PosVelAcc to the list
  /// @param[in] src the source pushing back to the list
  void push_back( const PosVelAcc& src ) {
    this->pvalist.push_back( src );
  };

  /// the source of PosVelAcc list
  std::vector<PosVelAcc> pvalist;
};

/////////////////////////////////////////////////////////////////////////////////////////

template
struct TimeVal<PVAList>;

/////////////////////////////////////////////////////////////////////////////////////////

typedef TimeVal<PVAList> TimePVAList;

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
  /// - SPLINE_SUCCESS: no error
  virtual RetCode push( const TimeVal<T>& newval );

  /// Push clock time and value data into buffer queue(FIFO)
  /// @param[in] clocktime  clock time of TimVal<T> value source
  /// @param[in] value      value of TimVal<T> value source
  /// @brief push time and value into the queue_buffer_
  /// @return
  /// - SPLINE_SUCCESS: no error
  virtual RetCode push_on_clocktime( const double& clocktime,
                                     const T& value );

  /// Push interval time(dT) and value data into buffer queue(FIFO)
  /// @param[in] dT    interval time of TimVal<T> value source
  /// @param[in] value value of TimVal<T> value source
  /// @brief push time from intervaltime and value into the queue_buffer_
  /// @return
  /// - SPLINE_SUCCESS: no error
  virtual RetCode push_on_dT( const double& dT,
                              const T& value );


  /// Pop T (oldest) data from buffer queue(FIFO)
  /// @brief delete the pop data from queue_buffer_
  /// @return output oldest T data
  /// @exception
  /// - SPLINE_QUEUE_SIZE_EMPTY: buffer size is not enough to pop.
  const TimeVal<T> pop();

  /// delete T front(oldest) data from buffer queue(FIFO)
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// @exception
  /// - SPLINE_QUEUE_SIZE_EMPTY: buffer size is not enough to pop and dlete.
   RetCode pop_delete();

  /// Get a value at the index
  /// @return constant a value at the index
  /// @exception If invalid index is accessed.
  const TimeVal<T> get( const std::size_t& index ) const
    throw(InvalidIndexAccess);

  /// Get a value at the first inputted index(oldest data)
  /// @return constant a value at the first index
  /// @exception If invalid index is accessed.
  const TimeVal<T> front() const
    throw(InvalidIndexAccess);


  /// Get a value at the last inputted index(newest data)
  /// @return constant a value at the last index
  /// @exception If invalid index is accessed.
  const TimeVal<T> back() const
    throw(InvalidIndexAccess);

  /// Set T value at the input-index
  /// @param[in] the index for setting T data
  /// @param[in] newval setting T new value
  /// @return setted T data
  /// - SPLINE_SUCCESS: No error
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index,
                       const TimeVal<T> newval );

  /// Clear all data of queue buffer
  void clear();

  /// Get queue size
  /// @return size of queue_buffer_
  const std::size_t size() const;

  // dump all queue list
  virtual RetCode dump( std::string& queue_dump );

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

  // dump all queue list
  virtual RetCode dump( std::string& queue_dump );

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
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const TimePVA& newval );

  /// Push TPV data into buffer queue(FIFO) (overload)
  /// @brief push TimePVA into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] pva  target PosVelAcc(position, velocity, acceleration)
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const PosVelAcc& pva );

  /// Push TPV data into buffer queue(FIFO) (overload)
  /// @brief push TimePVA into the tpv_buffer_
  /// @param[in] time         target time
  /// @param[in] position     target position
  /// @param[in] velocity     target velocity
  /// @param[in] acceleration target acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const double& position,
                const double& velocity,
                const double& acceleration );

  /// Set TimePVA value at the input-index (overload)
  /// @param[in] index     the index for setting TPV data
  /// @param[in] newval setting TimePVA value
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const TimePVA& newval );

  // dump all queue list
  virtual RetCode dump( std::string& queue_dump );

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
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const TimePVAList& newval );

  /// Push TimePVAList data into buffer queue(FIFO) (overload)
  /// @brief push time & PVAList into the tpv_buffer_
  /// @param[in] time     target time
  /// @param[in] pva_list target list of position, velocity, acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  RetCode push( const double& time,
                const PVAList& pva_list );

  /// Set TimePVAList value at the input-index (overload)
  /// @param[in] index     the index for setting TimePVA data
  /// @param[in] newval setting TimePVA value
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index,
                       const TimePVAList& newval );

  // dump all queue list
  virtual RetCode dump( std::string& queue_dump );

}; // End of class TPVAListQueue


/////////////////////////////////////////////////////////////////////////////////////////

/// Position & Velocity Queue buffer class
typedef std::deque<PosVelAcc> PVAQueue;

/////////////////////////////////////////////////////////////////////////////////////////

/// Base class of spline-path interpolator
class SplineInterpolator
{
public:
  /// Constructor
  SplineInterpolator();

  /// Destructor
  virtual ~SplineInterpolator();

  /// Get total interval time
  /// @return total interval time of spline-path
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double total_dT();

  /// Get limit of velocity( if exists )
  /// @return limit velocity
  /// @exception
  /// - NoVelocityLimit : velocity limit is not defined
  const double v_limit();

  /// Get finish v_limit( );

  /// Get finish time
  /// @return finish time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  const double finish_time();

  /// Genrate a spline-path from initial-finish point
  /// @param[in] xs start position
  /// @param[in] xf finish position
  /// @param[in] vs start velocity (default: 0.0)
  /// @param[in] vf finish velocity (default: 0.0)
  /// @param[in] as start acceleration (default: 0.0)
  /// @param[in] af finish acceleration (default: 0.0)
  /// @paran[in] ts start time (default: 0.0)
  /// @paran[in] tf finish time (default: 0.0)
  /// @brief calcurate tragectory parameter
  /// @return
  /// - SPLINE_SUCCESS
  /// @details
  /// If you don't give interval time(dT=0.0),
  /// Minmum interval time dT are internally calculated automatically. \n
  /// This means 100% mimum-time spline-path in the limitation.
  virtual RetCode generate_path( const double& xs, const double& xf,
                                 const double& vs=0.0, const double& vf=0.0,
                                 const double& as=0.0, const double& af=0.0,
                                 const double& ts=0.0, const double& df=0.0 );

  /// Generate a spline-path from Time, Position queue
  /// @param[in] target_tp_queue target Time,Position queue
  /// @param[in] vs              start velocity (default: 0.0)
  /// @param[in] vf              finish velocity (default: 0.0)
  /// @return
  /// - SPLINE_SUCCESS
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
  virtual RetCode generate_path( const TPQueue& target_tp_queue,
                                 const double vs=0.0, const double vf=0.0,
                                 const double as=0.0, const double af=0.0)=0;

  /// Generate a spline-path from Time, Position(, Velocity) queue
  /// @param[in] target_tpva_queue target Time, Position(, Velocity, Acceleration) queue
  /// @return
  /// - SPLINE_SUCCESS
  virtual RetCode generate_path( const TPVAQueue& target_tpva_queue )=0;

  /// Generate a spline-path from Position(, Velocity) queue
  /// @param[in] target_pv_queue target Position, Velocity queue
  /// @return
  /// - SPLINE_SUCCESS and total interval time (tf - ts)
  /// you only give start position & velocity & acceleration and don't give time. \n
  ///
  /// ```
  /// (xs, vs, as), (x1, v1, a1), (x2, v2, a2),..., (xf, vf, af)
  /// ```
  ///
  /// Minmum interval time -> ts=0, t1, t2,..., tf are internally calculated automatically. \n
  /// This means 100% mimum-time spline-path in the limitation.
  virtual RetCode generate_path( const PVAQueue& target_pva_queue )=0;

  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param[in] t input time
  /// @return output TPV at the input time
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  /// - TimeOutOfRange : time is not within the range of generated spline-path
  virtual const TimePVA pop( const double& t )=0;

protected:
  /// flag if the spline-path is generated. (default: false)
  bool is_path_generated_;

  /// flag if velocity limit (v_limit) is defined or not (default: false)
  bool is_v_limit_;

  /// limit velocity [m/s, rad/s, ...etc.]
  /// (if defined)
  double v_limit_;

  /// target TPVQueue
  TPVAQueue target_tpva_queue_;
}; // End of class SplineInterpolator

} // End of namespace interp
#endif // INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
