#ifndef INCLUDE_NON_UNIFORM_ROUNDING_SPLINE_HPP_
#define INCLUDE_NON_UNIFORM_ROUNDING_SPLINE_HPP_

#include "path_interpolator.hpp"

namespace interp {

/// Class of calculating velocity by Non-Uniform spline
class NonUniformRoundingSpline {
public:
  /// Constructor
  /// @param[in] init_position start position(initial value)
  /// @param[in] init_velocity start velocity(if necessary. default 0.0)
  /// @brief create initial state of time=0.0、position=input argument、velocity=0.
  NonUniformRoundingSpline(const double& init_position,
                           const double& init_velocity=0.0);

  /// Destructor
  ~NonUniformRoundingSpline();

  /// push(queue) the date next to the last index
  /// @param[in] time     target time
  /// @param[in] position target position
  /// @details
  /// If the queue has time-position data greater than or equal to 3 (>=3),
  /// this function calculates internally the interploated velocity
  /// by means of a non-Uniform rounding spline
  /// and adds a new time-position-Velocity data into the queue.
  void push(const double& time, const double& position);

  /// pop the Time-Position-Velocity data
  /// @param[out] output TimePVA(Time-Position-Velocity-(Acceleration~)) structure data.
  /// @brief
  /// pop the oldest Time-Position-Velocity data in the queue (FIFO)
  /// @return
  /// - PATH_SUCCESS : successfuly pop data.
  /// - PATH_QUEUE_SIZE_EMPTY : calculated buffer size is empty yet.
  RetCode pop(TimePVA& output);

  /// clear all data from the queue.
  /// @brief clear the all stored data from the queue.
  void clear();

  /// get the size of quque.
  /// @brief Returns the size of the stored Time-Position-Velocity buffer data.
  /// @return queue size.
  unsigned int size();

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
