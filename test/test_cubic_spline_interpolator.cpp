#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"
#include "test/util/test_graph_plot.hpp"


namespace interp {

/// generaed path from time-position queue
/// and generate cycletime-position-velocity queue from path
/// @param[in] target_tp target time-position queue
///                      (ts, target_tp_s),
///                      (t1, target_tp_1),
///                      ... ,
///                      (tf, target_tp_f)
/// @param[in] cycle     cycle time
/// @param[in] path      destination path of plotted graph image(.png)
/// @param[in] vs        start velocity (default=0.0)
/// @param[in] vf        finish velocity (default=0.0)
/// @return queue of interpolated time-position-velocity
const TPVAQueue g_generate_path_and_cycletime_queue( const TPQueue& target_tp,
                                                     const double cycle=0.005,
                                                     const double vs=0.0,
                                                     const double vf=0.0 ) {
  // cubic spline path interpolator
  CubicSplineInterpolator tg;
  // generate path & total dT
  if( tg.generate_path( target_tp, vs, vf ) != PATH_SUCCESS ) {
    THROW( UndefPathException, "failed to generated_path()");
  }
  double dT_total;
  tg.total_dT(dT_total);
  LOGD << "dT_total:" << dT_total;

  // queue into plot data
  TimePVA plot_point;
  TPVAQueue output_cycletime_tpva_queue;
  double finish_time;
  if( tg.finish_time( finish_time ) != PATH_SUCCESS) {
    THROW( UndefPathException, "failed to get finish_time() because of failing to generated_path()");
  }
  for(double t=target_tp.get(0).time; t<finish_time; t+=cycle) {
    tg.pop(t, plot_point);
    // add buffer of interpolated plot-point
    output_cycletime_tpva_queue.push( TimePVA( plot_point.time,
                                               plot_point.pos,
                                               plot_point.vel ) );
  }
  // add target point at last
  tg.pop( finish_time, plot_point );
  output_cycletime_tpva_queue.push( TimePVA( plot_point.time,
                                             plot_point.pos,
                                             plot_point.vel ) );
  return output_cycletime_tpva_queue;
};


/// Friend Test Class of CubicSplineInterpolator
class CubicSplineTest: public ::testing::Test
{
protected:
  CubicSplineTest() {}

  CubicSplineInterpolator cubic_spline_;

  RetCode m_tridiagonal_matrix_eq_solver(
    std::vector<double> d, const std::vector<double>& u,
    const std::vector<double>& l, std::vector<double> p,
    std::vector<double>& out_solved_x ) {
    //
    return cubic_spline_.tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x );
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
  std::vector<double> out_solved_x;

  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x ),
                InvalidArgumentSize );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_size ) {
  std::vector<double> d;
  std::vector<double> u;
  std::vector<double> l;
  std::vector<double> p;
  std::vector<double> out_solved_x;
  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x ),
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
  std::vector<double> out_solved_x;
  //
  RetCode retcode = m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x );
  EXPECT_EQ( retcode, PATH_INVALID_MATRIX_ARGUMENT_VALUE_ZERO );
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
  std::vector<double> out_solved_x;
  //
  RetCode retcode = m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x );
  EXPECT_EQ( retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[0], p[0] / d[0] ));
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
  std::vector<double> out_solved_x;
  //
  RetCode retcode = m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x );
  EXPECT_EQ( retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[0], 1.0/3.0 ) );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[1], -1.0/9.0 ) );
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
  std::vector<double> out_solved_x;
  //
  RetCode retcode = m_tridiagonal_matrix_eq_solver( d, u, l, p, out_solved_x );
  EXPECT_EQ( retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[0], 1.0/13.0 ) );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[1], 3.0/13.0 ) );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[2], -23.0/26.0 ) );
}

TEST_F( CubicSplineTest, pop1 ) {
  TPQueue tp_queue;
  tp_queue.push_on_dT( 0.0, -1.0 );
  tp_queue.push_on_dT( 1.0, -1.0 );
  tp_queue.push_on_dT( 2.0, 0.0 );
  tp_queue.push_on_dT( 3.0, 10.1 );
  tp_queue.push_on_dT( 4.0, 20.0 );
  tp_queue.push_on_dT( 5.0, 3.1 );
  tp_queue.push_on_dT( 6.0, 7.0 );
  tp_queue.push_on_dT( 7.0, 10.1 );

  // start velocity  = -0.0
  // finish velocity = 0.0
  TPVAQueue interpolated_path_tpva
    = g_generate_path_and_cycletime_queue(tp_queue, 0.005, -0.0, 0.0 );

  TestGraphPlot test_gp;
  test_gp.plot(tp_queue, interpolated_path_tpva, "./images/");
}
