#ifndef INCLUDE_SPLINE_DATA_HPP_
#define INCLUDE_SPLINE_DATA_HPP_

#include <typeinfo>
#include <math.h>
#include <vector>
#include <deque>
#include <cstddef> // for size_t
#include <iterator> // for back_inserter
#include <limits> // for PRECISION ( using epsilon )

#include "spline_exception.hpp"

namespace interp {

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

const double PRECISION = std::numeric_limits<double>::epsilon();

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
  /// @param[in] src source of copy which is type of PVAList
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
  /// @param[in] src the source of copy which is type of PVAList
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
  PosVelAcc& operator[]( const std::size_t& index ) {
    return this->pvalist[index];
  };

  /// @param[in] index the index of list
  /// @return the reference of PosVelAcc[index].pos
  double& pos( const std::size_t& index ) {
    return this->pvalist[index].pos;
  }

  /// @param[in] index index of list
  /// @return the reference of PosVelAcc[index].vel
  double& vel( const std::size_t& index ) {
    return this->pvalist[index].vel;
  }

  /// @param[in] index the index of list
  /// @return the reference of PosVelAcc[index].acc
  double& acc( const std::size_t& index ) {
    return this->pvalist[index].acc;
  }

  /// returns the list size of PVAList
  /// @return the list size of PVAList
  std::size_t size() {
    return this->pvalist.size();
  };

  /// resize resize of pvalist
  /// @param[in] size the size of resize
  void resize( const std::size_t& size ) {
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

/// Struct data of clock time, value
template<class T>
struct TimeVal {
  /// Constructor
  /// TimeVal():time(0.0) {};
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

// /// type definition of Time-Position struct data derived from TimeVal<T>
typedef TimeVal<double> TimePosition;

/////////////////////////////////////////////////////////////////////////////////////////

template
struct TimeVal<PosVelAcc>;

typedef TimeVal<PosVelAcc> TimePVA;

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
  virtual RetCode dump( const std::string& queue_dump );

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

/// TimePosition Queue buffer class
class TPQueue : public TimeQueue<double> {
public:
  /// Constructor
  TPQueue();

  /// Destructor
  virtual ~TPQueue();

  /// dump all queue list
  /// @param[out] dest_queue_dump output of all queue data as string.
  virtual RetCode dump( std::string& destt_queue_dump );

}; // End of class TPQueue

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
                const double& velocity=0.0,
                const double& acceleration=0.0 );

  /// Set TimePVA value at the input-index (overload)
  /// @param[in] index     the index for setting TPV data
  /// @param[in] newval setting TimePVA value
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  virtual RetCode set( const std::size_t& index, const TimePVA& newval );

  /// dump all queue list
  /// @param[out] dest_queue_dump output of all queue data as string.  
  virtual RetCode dump( std::string& dest_queue_dump );

}; // End of class TPVQueue

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

  /// dump all queue list
  /// @param[out] dest_queue_dump output of all queue data as string.
  virtual RetCode dump( std::string& dest_queue_dump );

}; // End of class TPVAListQueue


/////////////////////////////////////////////////////////////////////////////////////////

/// Position & Velocity Queue buffer class
typedef std::deque<PosVelAcc> PVAQueue;

/////////////////////////////////////////////////////////////////////////////////////////

/// 台形型5251525次軌道の構成パラメータ
struct TrapezoidConfig {
public:
  /// コンストラクタ
  TrapezoidConfig() :
    a_limit(1200), d_limit(1200),
    v_limit(170),
    asr(0.8), dsr(0.8),
    ratio_acc_dec(0.5) {
  };

  /// コンストラクタ
  /// @param[in] _a_limit 加速度(第一加速度)リミットの参照
  /// @param[in] _d_limit 減速度(第一減速度)リミットの参照
  /// @param[in] _v_limit 速度リミットの参照
  /// @param[in] _asr 加速側(第一加速側)丸め率の参照
  /// @param[in] _dsr 減速側(第二減速側)丸め率の参照
  /// @param[in] _ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率の参照
  TrapezoidConfig( const double& _a_limit,
                   const double& _d_limit,
                   const double& _v_limit,
                   const double& _asr,
                   const double& _dsr,
                   const double& _ratio_acc_dec );

  /// コピーコンストラクタ
  /// @param[in] src コピー元TrapezoidConfigの参照
  TrapezoidConfig( const TrapezoidConfig& src );

  /// 代入演算子
  /// @param[in] src コピー元TrapezoidConfig参照
  /// @return *athis
  TrapezoidConfig operator=( const TrapezoidConfig& src );

  /// 加速度(第一加速度)リミット
  double a_limit;
  /// 減速度(第一減速度)リミット
  double d_limit;
  /// 速度リミット
  double v_limit;
  /// 加速側(第一加速側)丸め率
  double asr;
  /// 減速側(第二減速側)丸め率
  double dsr;
  /// 第一＆第二加速度上限値に対する下限値の比率
  double ratio_acc_dec;
};

 /////////////////////////////////////////////////////////////////////////////////////////

/// 台形型5251525次軌道の構成パラメータのキュー
typedef std::deque<TrapezoidConfig> TrapezoidConfigQueue;

/////////////////////////////////////////////////////////////////////////////////////////


}; // End of class SplineInterpolator

#endif // end of INCLUDE_SPLINE_DATA_HPP_
