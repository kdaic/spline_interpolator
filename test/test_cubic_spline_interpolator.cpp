#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"
#include <plog/Log.h>
#include "test/util/gnuplot_realtime.hpp"

#include <fstream>


namespace interp {

/// path_generate ＆ plot graph
/// @param target_set time & position & velocity target
/// @param cycle cycle-time
/// @param path destination path of plotted graph image(.png)
/// @param vs start velocity (default=0.0)
/// @param vs finish velocity (default=0.0)
/// @details generate path of each set
/// plot graph of time-position, time-velocity, position-velocity \n
void plot( const TPQueue& target_set,
           const double cycle=0.005,
           const std::string& output_path="./",
           const double vs=0.0,
           const double vf=0.0 ) {
  // buffer of input target point
  std::deque<std::pair<double, double> > target_tp;
  std::deque<std::pair<double, double> > target_tv;
  std::deque<std::pair<double, double> > target_phase;
  // buffer of interpolated path point
  std::deque<std::pair<double, double> > path_tp;
  std::deque<std::pair<double, double> > path_tv;
  std::deque<std::pair<double, double> > path_phase;
  // option
  plot_options_t options_target;
  options_target["with"] = "point";
  options_target["ps"] = "5.0";
  options_target["pt"] = "3";
  options_target["lc"] = "rgb 'red'";
  plot_options_t options;
  options["with"] = "lp";
  // target point plot data
  std::vector<std::pair<double, double> >target_plot_data_tp;
  std::vector<std::pair<double, double> >target_plot_data_tv;
  std::vector<std::pair<double, double> >target_plot_data_phase;
  // interpolated point plot data
  std::vector<std::pair<double, double> >path_plot_data_tp;
  std::vector<std::pair<double, double> >path_plot_data_tv;
  std::vector<std::pair<double, double> >path_plot_data_phase;
  // input character into the graph
  std::stringstream numstr;
  // gnuplot
  GnuplotServer gpserver;
  // plot-data(time-target, time-interpolated-path)
  container_t multi_data;
  container_t multi_data_v;
  container_t multi_data_phase;
  // one set of time-posisiton-velocity
  TimePosition output;
  /// generated path & output for set of each path
  for(unsigned int index=0; index < 1; index++) {
    output = target_set.get(index);
    // generate_path
    CubicSplineInterpolator tg;
    RetVal<double> ret_dT_total = tg.generate_path( target_set, vs, vf );
    double dT_total = ret_dT_total.value;
    LOGD << "dT_total:" << dT_total;

    for(unsigned int i=0; i < target_set.size(); i++) {
      TimePosition tg_output = target_set.get(i);
      target_tp.push_back(    std::make_pair(tg_output.time,     tg_output.position) ); // time-position
      // target_tv.push_back(    std::make_pair(tg_output.time,     0.0) ); // time-velocity
      // target_phase.push_back( std::make_pair(tg_output.position, 0.0) ); // position-velocity
    }

    // prepare output data log
    // set 2 digit lefty embeded 0
    numstr << "images/"
           << std::setfill('0') << std::setw(2)
           << index << "_time_position_velocity.csv";
    std::fstream cfstrm( numstr.str().c_str(), std::ios::out );
    numstr.str("");
    if (cfstrm.fail()) {
      std::cerr << "cannot open output temporary file."<< std::endl;
      FAIL();
    }
    // fix digit
    cfstrm << std::fixed << std::setprecision(8);

    // queue into plot data
    TPV plot_point;
    for(double t=output.time; t < tg.finish_time().value; t+=cycle) {
      plot_point = tg.pop(t).value;
      /////////////////////////////////////////////////////////////////////////
      /// output time-position-velocity data file
      /////////////////////////////////////////////////////////////////////////
      {
        cfstrm << t
               << "," << plot_point.position
               << "," << plot_point.velocity << std::endl;
      }
      // add buffer of interpolated plot-point
      path_tp.push_back(    std::make_pair(t,                   plot_point.position) );
      path_tv.push_back(    std::make_pair(t,                   plot_point.velocity) );
      path_phase.push_back( std::make_pair(plot_point.position, plot_point.velocity) );
    }
    // add target point data at last
    plot_point = tg.pop( tg.finish_time().value ).value;
    {
      cfstrm << plot_point.time
             << "," << plot_point.position
             << "," << plot_point.velocity << std::endl;
    }
    cfstrm.close();
    path_tp.push_back(    std::make_pair(tg.finish_time().value, plot_point.position) );
    path_tv.push_back(    std::make_pair(tg.finish_time().value, plot_point.velocity) );
    path_phase.push_back( std::make_pair(plot_point.position,    plot_point.velocity) );

    // add buffer time-position plot
    for(std::deque<std::pair<double, double> >::iterator it = target_tp.begin();
        it < target_tp.end();
        it++) {
      target_plot_data_tp.push_back(*it);
    }
    for(std::deque<std::pair<double, double> >::iterator it = path_tp.begin();
        it < path_tp.end();
        it++) {
      path_plot_data_tp.push_back(*it);
    }
    multi_data.push_back( std::make_pair(target_plot_data_tp, options_target) );
    multi_data.push_back( std::make_pair(path_plot_data_tp,   options) );

    // add buffer time-veelocity plot
    for(std::deque<std::pair<double, double> >::iterator it = target_tv.begin();
        it < target_tv.end();
        it++) {
      target_plot_data_tv.push_back(*it);
    }
    for(std::deque<std::pair<double, double> >::iterator it = path_tv.begin();
        it < path_tv.end();
        it++) {
      path_plot_data_tv.push_back(*it);
    }
    multi_data_v.push_back( std::make_pair(target_plot_data_tv, options_target) );
    multi_data_v.push_back( std::make_pair(path_plot_data_tv,   options) );

    // 位置-速度描画のバッファ追加
    for(std::deque<std::pair<double, double> >::iterator it = target_phase.begin();
        it < target_phase.end();
        it++) {
      target_plot_data_phase.push_back(*it);
    }
    for(std::deque<std::pair<double, double> >::iterator it = path_phase.begin();
        it < path_phase.end();
        it++) {
      path_plot_data_phase.push_back(*it);
    }
    multi_data_phase.push_back( std::make_pair(target_plot_data_phase, options_target) );
    multi_data_phase.push_back( std::make_pair(path_plot_data_phase,   options) );

    /////////////////////////////////////////////////////////////////////////
    /// time-position plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2) << index << "_time-position_graph.png'";
    std::cerr << "drawing " << numstr.str() << "..." << std::endl;
    gpserver.set("output " +  numstr.str());
    numstr.str("");
    gpserver.set("style data lp");
    gpserver.set("xzeroaxis");
    gpserver.set("grid");
    // gpserver.set("noautoscale");
    gpserver.set("xlabel 'time'");
    gpserver.set("ylabel 'position'");
    gpserver.set("nokey");
    gpserver.flush();
    // plot
    gpserver.plot(multi_data);
    gpserver.flush();
    // clear
    target_tp.clear();
    path_tp.clear();
    target_plot_data_tp.clear();
    path_plot_data_tp.clear();
    multi_data.clear();

    /////////////////////////////////////////////////////////////////////////
    /// time-velocity plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2) << index << "_time-velocity_graph.png'";
    std::cerr << "drawing " << numstr.str() << "..." << std::endl;
    gpserver.set("output " +  numstr.str());
    numstr.str("");
    gpserver.set("style data lp");
    gpserver.set("xzeroaxis");
    gpserver.set("grid");
    // gpserver.set("noautoscale");
    gpserver.set("xlabel 'time'");
    gpserver.set("ylabel 'velocity'");
    gpserver.set("nokey");
    gpserver.flush();
    // plot
    gpserver.plot(multi_data_v);
    gpserver.flush();
    // clear
    target_tv.clear();
    path_tv.clear();
    target_plot_data_tv.clear();
    path_plot_data_tv.clear();
    multi_data_v.clear();

    /////////////////////////////////////////////////////////////////////////
    /// position-velocity plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2) << index << "_phase_diagram.png'";
    std::cerr << "drawing " << numstr.str() << "..." << std::endl;
    gpserver.set("output " +  numstr.str());
    numstr.str("");
    gpserver.set("style data lp");
    gpserver.set("xzeroaxis");
    gpserver.set("grid");
    // gpserver.set("noautoscale");
    gpserver.set("xlabel 'position'");
    gpserver.set("ylabel 'velocity'");
    gpserver.set("nokey");
    gpserver.flush();
    // plot
    gpserver.plot(multi_data_phase);
    gpserver.flush();
    // clear
    target_phase.clear();
    path_phase.clear();
    target_plot_data_phase.clear();
    path_plot_data_phase.clear();
    multi_data_phase.clear();
  }
  usleep(0.01*1e6);
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

}

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
  // plot(tp_queue, 0.005, "./images/");
  plot(tp_queue, 0.005, "./images/", -15, 15);
}
