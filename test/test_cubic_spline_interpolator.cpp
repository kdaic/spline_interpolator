#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"
#include "test/util/test_graph_plot.hpp"


namespace interp {

/// generaed spline-path from time-position queue
/// and generate cycletime-position-velocity queue from interpolated path
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
  if( tg.generate_path( target_tp, vs, vf ) != SPLINE_SUCCESS ) {
    THROW( UndefSplineException,
           "failed to generated_path()");
  }

  // queue into plot data

  TimePVA plot_point;
  TPVAQueue output_cycletime_tpva_queue;
  for(double t=target_tp.get(0).time; t<tg.finish_time(); t+=cycle) {
    plot_point = tg.pop( t );
    // add buffer of interpolated plot-point
    output_cycletime_tpva_queue.push( TimePVA( plot_point.time,
                                               PosVelAcc( plot_point.P.pos,
                                                          plot_point.P.vel ) ) );
  }
  // add target point at last
  plot_point = tg.pop( tg.finish_time() );
  output_cycletime_tpva_queue.push( TimePVA( plot_point.time,
                                             PosVelAcc( plot_point.P.pos,
                                                        plot_point.P.vel ) ) );
  return output_cycletime_tpva_queue;
};


/// Friend Test Class of CubicSplineInterpolator
class CubicSplineTest: public ::testing::Test {
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
  EXPECT_EQ( retcode, SPLINE_INVALID_MATRIX_ARGUMENT_VALUE_ZERO );
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
  EXPECT_EQ( retcode, SPLINE_SUCCESS );
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
  EXPECT_EQ( retcode, SPLINE_SUCCESS );
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
  EXPECT_EQ( retcode, SPLINE_SUCCESS );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[0], 1.0/13.0 ) );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[1], 3.0/13.0 ) );
  EXPECT_TRUE( g_isNearlyEq( out_solved_x[2], -23.0/26.0 ) );
}

TEST_F( CubicSplineTest, pop1 ) {

#ifndef __QNX__

  TPQueue tp_queue; // TP = time, position
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

  /// ディレクトリ作成
  const std::string output_dir = "./images/cubic_spline";
  const std::string mkdir_outdir_str = "mkdir -p " + output_dir;
  FILE* mkdir_outdir = popen(mkdir_outdir_str.c_str(), "re");
  pclose(mkdir_outdir);

  // 既に存在する画像を削除
  const std::string rm_outdir_images_str = "rm -f " + output_dir + "/*.png";
  FILE* rm_outdir_images = popen(rm_outdir_images_str.c_str(), "re");
  pclose(rm_outdir_images);

  TestGraphPlot test_gp;
  test_gp.plot_tp_tv_pv( tp_queue,
                         interpolated_path_tpva,
                         output_dir );
  test_gp.dump_csv( interpolated_path_tpva, output_dir );
#endif // #ifdef __QNX__

}


TEST_F( CubicSplineTest, index_of_time ) {

  TPQueue tp_queue; // TP = time, position
  tp_queue.push_on_clocktime( 0.0, -1.0 );
  tp_queue.push_on_clocktime( 1.0, -1.0 );
  tp_queue.push_on_clocktime( 2.0, 0.0 );
  tp_queue.push_on_clocktime( 3.0, 10.1 );
  tp_queue.push_on_clocktime( 4.0, 20.0 );
  tp_queue.push_on_clocktime( 5.0, 3.1 );
  tp_queue.push_on_clocktime( 6.0, 7.0 );
  tp_queue.push_on_clocktime( 7.0, 10.1 );

   // cubic spline path interpolator
  CubicSplineInterpolator tg;

  // start velocity  = -0.0
  double vs = 0.0;
  // finish velocity = 0.0
  double vf = 0.0;
  // generate path & total dT
  if( tg.generate_path( tp_queue, vs, vf ) != SPLINE_SUCCESS ) {
    THROW( UndefSplineException,
           "failed to generated_path()");
  }

  // test of index_of_time
  std::size_t index=100;
  EXPECT_EQ( SPLINE_INVALID_INPUT_TIME, tg.index_of_time(-1.0, index) );
  EXPECT_EQ( 100, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(0.0, index) );
  EXPECT_EQ( 0, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(0.5, index) );
  EXPECT_EQ( 0, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(1.0, index) );
  EXPECT_EQ( 1, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(1.5, index) );
  EXPECT_EQ( 1, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(2.0, index) );
  EXPECT_EQ( 2, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(2.5, index) );
  EXPECT_EQ( 2, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(6.5, index) );
  EXPECT_EQ( 6, index );
  EXPECT_EQ( SPLINE_SUCCESS, tg.index_of_time(7.0, index) );
  EXPECT_EQ( 7, index );
  index=100;
  EXPECT_EQ( SPLINE_INVALID_INPUT_TIME, tg.index_of_time(7.1, index) );
  EXPECT_EQ( 100, index );
}
