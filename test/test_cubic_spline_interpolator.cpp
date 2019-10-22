#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"
#include "test/util/test_graph_plot.hpp"


namespace interp {

/// generaed path from time-position queue
/// @param target_set target time-position queue
///        (ts, target_tp_s), (t1, target_tp_1), ... , (tf, target_tp_f)
/// @param cycle cycle time
/// @param path destination path of plotted graph image(.png)
/// @param vs start velocity (default=0.0)
/// @param vs finish velocity (default=0.0)
/// @return queue of interpolated time-position-velocity
const TPVQueue g_generate_path_queue( const TPQueue& target_tp,
                                      const double cycle=0.005,
                                      const double vs=0.0,
                                      const double vf=0.0 ) {
  // cubic spline path interpolator
  CubicSplineInterpolator tg;
  // generate path & total dT
  RetVal<double> ret_dT_total = tg.generate_path( target_tp, vs, vf );
  double dT_total = ret_dT_total.value;
  LOGD << "dT_total:" << dT_total;

  // queue into plot data
  TPV plot_point;
  TPVQueue output_path_tpv_queue;
  for(double t=target_tp.get(0).time; t < tg.finish_time().value; t+=cycle) {
    plot_point = tg.pop(t).value;
    // add buffer of interpolated plot-point
    output_path_tpv_queue.push(
                            TPV( plot_point.time,
                                 plot_point.position,
                                 plot_point.velocity ) );
  }
  // add target point at last
  plot_point = tg.pop( tg.finish_time().value ).value;
  output_path_tpv_queue.push(
                          TPV( plot_point.time,
                               plot_point.position,
                               plot_point.velocity ) );
  return output_path_tpv_queue;
};


/// Friend Test Class of CubicSplineInterpolator
class CubicSplineTest: public ::testing::Test
{
protected:
  CubicSplineTest() {}

  CubicSplineInterpolator cubic_spline_;

  RetVal<std::vector<double> >
  m_tridiagonal_matrix_eq_solver( std::vector<double> d, const std::vector<double>& u,
                                const std::vector<double>& l, std::vector<double> p ) {
    return cubic_spline_.tridiagonal_matrix_eq_solver( d, u, l, p );
  }
};

} // end of namespace interp

using namespace interp;

TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_size_not_same ) {
  double d_array[] = {4, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1};
  std::vector<double> p(p_array, p_array + 1);

  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p ),
                InvalidArgumentSize );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_size ) {
  std::vector<double> d;
  std::vector<double> u;
  std::vector<double> l;
  std::vector<double> p;
  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p ),
                InvalidArgumentSize );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_value_zero ) {
  double d_array[] = {0, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1, 0};
  std::vector<double> p(p_array, p_array + 2);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_INVALID_ARGUMENT_VALUE_ZERO );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_list_size_is_1 ) {
  double d_array[] = {4};
  std::vector<double> d(d_array, d_array + 1);
  double u_array[] = {3};
  std::vector<double> u(u_array, u_array + 1);
  double l_array[] = {2};
  std::vector<double> l(l_array, l_array + 1);
  double p_array[] = {1};
  std::vector<double> p(p_array, p_array + 1);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], p[0] / d[0] ));
}

TEST_F( CubicSplineTest, tri_matrix_eq_solver_size_is_2 ) {
  double d_array[] = {4, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1, 0};
  std::vector<double> p(p_array, p_array + 2);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], 1.0/3.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[1], -1.0/9.0 ) );
}

TEST_F( CubicSplineTest, tri_matrix_eq_solver_size_is_3 ) {
  double d_array[] = {4, 3, 2};
  std::vector<double> d(d_array, d_array + 3);
  double u_array[] = {3, 2, 1};
  std::vector<double> u(u_array, u_array + 3);
  double l_array[] = {2, 1, -1};
  std::vector<double> l(l_array, l_array + 3);
  double p_array[] = {1, -1, -2};
  std::vector<double> p(p_array, p_array + 3);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], 1.0/13.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[1], 3.0/13.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[2], -23.0/26.0 ) );
}

TEST_F( CubicSplineTest, pop1 ) {
  TPQueue tp_queue;
  tp_queue.push( 0.0, -1.0 );
  tp_queue.push( 1.0, -1.0 );
  tp_queue.push( 2.0, 0.0 );
  tp_queue.push( 3.0, 10.1 );
  tp_queue.push( 4.0, 20.0 );
  tp_queue.push( 5.0, 3.1 );
  tp_queue.push( 6.0, 7.0 );
  tp_queue.push( 7.0, 10.1 );

  // start velocity  = -0.0
  // finish velocity = 0.0
  TPVQueue interpolated_path_tpv
    = g_generate_path_queue(tp_queue, 0.005, -0.0, 0.0 );

  TestGraphPlot test_gp;
  test_gp.plot(tp_queue, interpolated_path_tpv, "./images/");
}
