#ifndef INCLDE_CUBIC_SPLINE_INTERPOLATOR_
#define INCLDE_CUBIC_SPLINE_INTERPOLATOR_
#include "path_interpolator.hpp"

namespace interp{
/// Cubic Spline Interpolator
/// @brief parameters are kept internally.
/// @details Cubic spline is defined as.
///
/// ```
/// x(t) = a t^3 + b t^2 + c t + d
/// ```
class CubicSplineInterpolator : public PathInterpolator {
  friend class CubicSplineTest;
public:
  /// Constructor
  CubicSplineInterpolator();

  /// Destructor
  ~CubicSplineInterpolator();

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
  /// Interpolator set start & finish velocity, acceleration,(and jerk) as zero,
  ///
  /// ```
  /// (ts, xs, vs=0, as=0, js=0),
  /// (t1, x1, v1=?, a1=?, j1=?),
  /// (t2, x2, v2=?, a2=?, j2=?),
  ///  ...,
  /// (tf, xf, vf=0, af=0, jf=0)
  /// ```
  ///
  /// and interpolate (v1,a1,j1), (v1,a1,j1),.. automatically.
  virtual RetVal<double> generate_path( const TPQueue& tp_queue );

  /// Generate a path from Time, Position(, Velocity) queue
  /// @param[in] Time,Position(, Velocity) queue
  /// @return
  /// - PATH_SUCCESS and total travel time (tf - ts)
  virtual RetVal<double> generate_path( const TPVQueue& tpv_queue );


  /// Pop the position and velocity at the input-time from generated tragectory
  /// @param t input time
  /// - PATH_SUCCESS & TPV at the input time
  /// - PATH_NOT_GENERATED and TPV is time=-1.0, position=0.0, velocity=0.0
  virtual RetVal<TPV> pop(const double& t );

private:
  /// Tridiagonal Matrix Equation Solver
  /// @param[in] d diagonal elements list
  /// @param[in] u upper elements list
  /// @param[in] l lower elements list
  /// @param[in] p pushed out parameters list of Matrix Conversion
  /// @return x solved list of tridiagonal matrix equation
  /// @details
  /// Input Parameters d, u, l, p lists are assined in following Matrix eq.
  ///
  /// ```
  /// ┌                                                          ┐┌     ┐   ┌     ┐
  /// │ d_0  u_0    0                                            ││ x_0 │   │ p_0 │
  /// │ l_1  d_1  u_1                                            ││ x_1 │   │ p_1 │
  /// │      l_2  d_2 u_2                                        ││ x_2 │   │ p_2 │
  /// │           . . . . . .                                    ││  .  │   │  .  │
  /// │               l_k-1  d_k-1 u_k-1                         ││x_k-1│ = │p_k-1│
  /// │                        l_k   d_k   u_k                   ││ x_k │   │ p_k │
  /// │                            l_k+1 d_k+1 u_k+1             ││x_k+1│   │p_k+1│
  /// │                                   . . . . . .            ││  .  │   │  .  │
  /// │                                        l_N-1 d_N-1 u_N-1 ││x_N-1│   │p_N-1│
  /// │                                            0   l_N   d_N ││ x_N │   │ p_N │
  /// └                                                          ┘└     ┘   └     ┘
  /// ```
  ///
  /// This solver solved Output x lists. \n
  /// If the size of list is 1, Solver expects following Matrix eq.
  ///
  /// ```
  /// [ d_0 ][x_0] = [p_0]
  /// ```
  ///
  /// If the size of list is 2, Solver expects following Matrix eq.
  ///
  /// ```
  /// ┌         ┐┌     ┐   ┌     ┐
  /// │ d_0 u_0 ││ x_0 │ = │ p_0 │
  /// │ l_1 d_1 ││ x_1 │   │ p_1 │
  /// └         ┘└     ┘   └     ┘
  /// ```
  RetVal<std::vector<double> >
  tridiagonal_matrix_eq_solver( std::vector<double> d, const std::vector<double>& u,
                                const std::vector<double>& l, std::vector<double> p);
};

}

#endif // INCLDE_CUBIC_SPLINE_INTERPOLATOR_HPP_
