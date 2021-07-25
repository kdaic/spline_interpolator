#ifndef INCLUDE_NON_UNIFORM_ROUNDING_SPLINE_HPP_
#define INCLUDE_NON_UNIFORM_ROUNDING_SPLINE_HPP_

#include "spline_interpolator.hpp"

namespace interp {

/// Class of calculating velocity by Non-Uniform spline
class NonUniformRoundingSpline {
public:
  /// Constructor
  NonUniformRoundingSpline();

  /// Copy Constructor
  /// @param[in] src source of NonUniformRoundingSpline object,
  NonUniformRoundingSpline( const NonUniformRoundingSpline& src );

  /// Constructor
  /// @param[in] init_position start position(initial value)
  /// @param[in] init_velocity start velocity(if necessary. default 0.0)
  /// @brief create initial state of time=0.0、position=input argument、velocity=0.
  NonUniformRoundingSpline(const double& init_position,
                           const double& init_velocity=0.0);

  /// Destructor
  ~NonUniformRoundingSpline();

  /// copy operator
  /// @param[in] src source of NonUniformRoundingSpline object,
  /// @return *this
  NonUniformRoundingSpline operator=( const NonUniformRoundingSpline& src );

  /// velocity calculating
  /// @param[in] tp0 time-position,velocity,accerlation point0.
  /// @param[in] tp1 time-position,velocity,accerlation point1.
  ///                this is target to calurate & get veloctiy.
  /// @param[in] tp2 time-position,velocity,accerlation point2.
  /// @return velocity
  double calculate_velocity( const TimePVA& tp0,
                             const TimePVA& tp1,
                             const TimePVA& tp2 );

  /// push(queue) the data next to the last index
  /// @param[in] clock_time  target clock time
  /// @param[in] position    target position
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// adds a new time-position-Velocity data into the queue with velocity no changed.
  RetCode push_without_velocity_change( const double& clock_time,
                                        const double& position );

  /// push(queue) the data next to the last index
  /// @param[in] time_pva  target time-position-velocity-acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// adds a new time-position-Velocity data into the queue with velocity no changed.
  RetCode push_without_velocity_change( const TimePVA& time_pva );

  /// push(queue) the data next to the last index
  /// @param[in] clock_time  target clock time
  /// @param[in] position    target position
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  RetCode push( const double& clock_time, const double& position) ;

  /// push(queue) the data next to the last index
  /// @param[in] time_pva target time-position-velocity-acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  RetCode push( const TimePVA& time_pva );

  /// push(queue) the data next to the last index
  /// @param[in] interval_time  interval time between now and post target
  /// @param[in] position       target position
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// adds a new time-position-Velocity data into the queue with velocity no changed.
  RetCode push_dT_without_velocity_change(
            const double& interval_time, const double& position );

  /// push(queue) the data next to the last index
  /// @param[in] interval_time  interval time between now and post target
  /// @param[in] pva            target position-velocity-acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// adds a new time-position-Velocity data into the queue with velocity no changed.
  RetCode push_dT_without_velocity_change(
            const double& interval_time, const PosVelAcc& pva );

  /// push(queue) the data next to the last index
  /// @param[in] interval_time  interval time between now and post target
  /// @param[in] position       target position
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  RetCode push_dT( const double& interval_time, const double& position );

  /// push(queue) the data next to the last index
  /// @param[in] interval_time  interval time between now and post target
  /// @param[in] pva            target position-velocity-acceleration
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_TIME: the time is less than the one of previous index
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  RetCode push_dT( const double& interval_time, const PosVelAcc& pva );

  /// set Time-Position-Velocity-Acceleration to the index node
  /// @param[in] index    target index
  /// @param[in] velocity target velocity
  /// @return
  /// - SPLINE_SUCCESS: no error
  /// - SPLINE_INVALID_INPUT_INDEX: Not exist input-index
  RetCode force_set_velocity( const std::size_t& index , const double& velocity );

  /// get the tpva_buffer_
  /// @return tpva_buffer_
  const TPVAQueue tpva_queue() const;

  /// get the TimePVA at the index of the tpva_buffer_
  /// @return TimePVA reference at the index of tpva_buffer_
  const TimePVA get( const std::size_t& index ) const;

  /// get the TimePVA at the last index of the tpva_buffer_
  /// @return TimePVA reference at the last index of tpva_buffer_
  const TimePVA front() const;

  /// get the TimePVA at the first index of the tpva_buffer_
  /// @return TimePVA reference at the first index of tpva_buffer_
  const TimePVA back() const;

  /// pop the Time-Position-Velocity data
  /// @param[out] output ouput TimePVA(Time-Position-Velocity-(Acceleration=0)) structure data.
  /// @brief
  /// pop the oldest Time-Position-Velocity data in the queue (FIFO)
  /// @return
  /// - SPLINE_QUEUE_SIZE_EMPTY : calculated buffer size is empty yet.
  const TimePVA pop();

  /// pop back the Time-Position-Velocity data
  /// @param[out] output ouput TimePVA(Time-Position-Velocity-(Acceleration=0)) structure data.
  /// @brief
  /// pop the newest Time-Position-Velocity data in the queue (LO)
  /// @return
  /// - SPLINE_QUEUE_SIZE_EMPTY : calculated buffer size is empty yet.
  const TimePVA pop_back();

  /// clear all data from the queue.
  /// @brief clear the all stored data from the queue.
  void clear();

  /// get the size of quque.
  /// @brief Returns the size of the stored Time-Position-Velocity buffer data.
  /// @return queue size.
  unsigned int size();

  /// Get interval time at the input-index
  /// @param[in] index the index getting dT from queue
  /// @return dT at the input-index
  /// @exception If invalid index is accessed.
  const double dT( const std::size_t& index ) const
        throw(InvalidIndexAccess);

  /// Get total interval time summarized each dT in tpva_queue
  /// @return total_dT();
  const double total_dT() const;

private:

  /// time, position, calculated velocity.
  TPVAQueue tpva_buffer_;
};

} // End of namespace interp

#endif // INCLUDE_NON_UNIFORM_SPLINE_HPP_
