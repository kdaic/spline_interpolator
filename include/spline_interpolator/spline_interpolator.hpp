#ifndef INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
#define INCLUDE_BASE_SPLINE_INTERPOLATOR_HPP_
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


namespace interp {
// spline intepolator

/// Exception

/// struct data of Time,Position,Velocity
struct TPV {
public:
  /// constructor
  TPV(){};

  /// constructor(data copy)
  TPV(const double& _time=0.0, const double& _position=0.0, const double& _velocity=0.0):
    time(_time), position(_position), velocity(_velocity) {};

  /// destructor
  ~TPV(){};

  /// copy operator
  TPV operator=(const TPV& src) {
    TPV dest(src.time, src.position, src.velocity);
    this->time = dest.time;
    this->position = dest.position;
    this->velocity = dest.velocity;
    return *this;
  };
  ///
  double time;
  ///
  double position;
  ///
  double velocity;
};


/// TPV Queue buffer base-class
class TPVQueue {
  /// constructor
  /// @param[in] init_position start position
  /// @param[in] init_velocity start velocity
  /// @brief initial time is zero(0.0)
  TPVQueue( const double& init_position,
            const double& init_velocity=0.0);

  /// destructor
  virtual ~TPVQueue();

  /// copy operator
  TPV operator=(const TPVQueue& src);


  /// push TPV data into buffer queue(FIFO)
  /// @brief push TPV into the tpv_buffer_
  /// @param[in] time target time
  /// @param[in] position target position
  /// @exception the time is less than the one of previous index
  virtual void push( const double& time,
                     const double& position,
                     const double& velocity=0.0 );

  /// pop TPV data from buffer queue(FIFO)
  /// @brief delete the pop data from the tpv_buffer_
  /// @return oldest TPV data
  virtual TPV pop();

  /// get TPV value at the index
  /// @return constant TPV
  const TPV get( const std::size_t index );

  /// set TPV value at the input-index
  /// @param[in] the index for setting TPV data
  /// @param[in] tpv_val setting TPV value
  /// @return setted TPV data
  /// @throw the time is less than the one of previous index
  /// @throw not exist input-index
  const TPV set( const std::size_t index, const TPV& tpv_val );

  /// clear all data of buffer
  void clear();

  /// get queue size
  /// @return size of tpv_buffer_
  const std::size_t size();

private:
  /// the buffer of time,position,velocity
  std::deque<TPV> tpv_buffer_;
};


/////////////////////////////////////////////////////////////////////////////////////////

/// base class of spline interpolator
class BaseSplineInterpolator
{
public:

  /// constructor
  BaseSplineInterpolator();

  /// destructor
  virtual ~BaseSplineInterpolator();

  /// genrate tragectory path from initial-finish point
  /// @param[in] x0 initial position
  /// @param[in] v0 initial velocity
  /// @param[in] xf final position
  /// @param[in] vf final velocity (default: 0.0)
  /// @param[in] t0 initial time (default: 0.0)
  /// @paran[in] tf final time (default: -1.0)
  /// @brief calcurate tragectory parameter
  /// @return total travel time (tf - t0)
  virtual double generate_path( const double& x0, const double& v0,
                                const double& xf, const double& vf,
                                const double& t0=0.0, const double& tf=0.0 );

  /// generate tragectory path from Time,Position(,Velocity) queue
  /// @param[in] Time,Position(,Velocity) queue
  /// @return total travel time (tf - t0)
  virtual double genrate_path( const TPVQueue& tpv_queue );

  /// pop the position and velocity at the input-time from generated tragectory
  /// @param t input-time
  /// @param xt the position at the input-time
  /// @param vt the velocity at the input-time
  /// @return 0(if succeed)
  /// @exception generate_path() has not done.
  virtual int pop(const double& t, double& xt, double& vt);

  /// get final-time
  /// @return tf_
  virtual const double final_time();

  /// get limit velocity( if exists )
  /// @return limit velocity
  virtual const double v_limit();
};

} // end of namespace interp
#endif // INCLUDE_BASE_SPLINE_INTERPOLATOR_
