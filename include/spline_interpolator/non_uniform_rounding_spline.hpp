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

  /// push(queue) the date next to the last index
  /// @param[in] clock_time  target clock time
  /// @param[in] position    target position
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  void push(const double& clock_time, const double& position);

  /// push(queue) the date next to the last index
  /// @param[in] interval_time  interval time between now and post target
  /// @param[in] position       target position
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  void push_dT(const double& interval_time, const double& position);

  /// get the tpva_buffer_
  /// @return tpva_buffer_
  const TPVAQueue tpva_queue() const;

  /// get the TimePVA at the index of the tpva_buffer_
  /// @return TimePVA reference at the index of tpva_buffer_
  const TimePVA get( const std::size_t& index ) const;

  /// pop the Time-Position-Velocity data
  /// @param[out] output ouput TimePVA(Time-Position-Velocity-(Acceleration=0)) structure data.
  /// @brief
  /// pop the oldest Time-Position-Velocity data in the queue (FIFO)
  /// @return
  /// - SPLINE_QUEUE_SIZE_EMPTY : calculated buffer size is empty yet.
  const TimePVA pop();

  /// clear all data from the queue.
  /// @brief clear the all stored data from the queue.
  void clear();

  /// get the size of quque.
  /// @brief Returns the size of the stored Time-Position-Velocity buffer data.
  /// @return queue size.
  unsigned int size();

  /// Get dT at the input-index
  /// @param[in] index the index getting dT from queue
  /// @return dT at the input-index
  /// @exception If invalid index is accessed.
  const double dT( const std::size_t& index ) const;


private:
  /// velocity calculating
  /// @return velocity
  double calculate_velocity();

  /// inputted time, position
  TPQueue tp_buffer_;

  /// time, position, calculated velocity.
  TPVAQueue tpva_buffer_;
};

} // End of namespace interp

#endif // INCLUDE_NON_UNIFORM_SPLINE_HPP_
