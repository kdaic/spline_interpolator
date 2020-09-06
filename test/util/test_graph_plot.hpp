#ifndef INCLUDE_TEST_GRAPH_PLOT_HPP_
#define INCLUDE_TEST_GRAPH_PLOT_HPP_

#include <plog/Log.h>
#include "path_interpolator.hpp"
#include "test/util/gnuplot_realtime.hpp"

#include <fstream>

namespace interp {

/// graph plot class
class TestGraphPlot {
public:
  /// constructor
  TestGraphPlot() {};
  /// destructor
  ~TestGraphPlot() {};
  /// plot graph (overload)
  /// @param[in] target_tp       plotting target time-position data
  ///                            (ts, target_tp_s),
  ///                            (t1, target_tp_1),
  ///                            ... ,
  ///                            (tf, target_tp_f)
  /// @param[in] interp_path_tpv plotting interpolated path
  ///                            of time-position-velocity with cycletime dT.
  ///                            (ts,     path(s)),
  ///                            (ts+dT,  path(ts+dT)),
  ///                            (ts+2dT, path(ts+2dT)),
  ///                            ... ,
  ///                            (tf,     path(tf))
  /// @param[in] output_path     destination path of plotted graph image(.png)
  /// @param[in] path_index      prefix index of destination path of plotted graph
  void plot( const TPQueue& target_tp,
             const TPVQueue& interp_path_tpv,
             const std::string& output_path="./",
             const int path_index = 0 ) {
    TPVQueue empty_target_tpv;
    plot( target_tp,
          empty_target_tpv,
          interp_path_tpv,
          output_path,
          path_index );
  }

  /// plot graph (overload)
  /// @param[in] target_tpv      plotting target time-position-velocity data
  ///                            (ts, target_tpv_s),
  ///                            (t1, target_tpv_1),
  ///                             ... ,
  ///                            (tf, target_tpv_f)
  /// @param[in] interp_path_tpv plotting interpolated path
  ///                            of time-position-velocity with cycletime dT.
  ///                            (ts,     path(s)),
  ///                            (ts+dT,  path(ts+dT)),
  ///                            (ts+2dT, path(ts+2dT)),
  ///                            ... ,
  ///                            (tf,     path(tf))
  /// @param[in] output_path     destination path of plotted graph image(.png)
  /// @param[in] path_index      prefix index of destination path of plotted graph
  void plot( const TPVQueue& target_tpv,
             const TPVQueue& interp_path_tpv,
             const std::string& output_path="./",
             const int path_index = 0 ) {
    TPQueue target_tp;
    for(std::size_t i=0; i<target_tpv.size(); i++) {
      // add target of time-velocity into plotting buffer
      target_tp.push(
                     TimePosition( target_tpv.get(i).time,
                                   target_tpv.get(i).position ) );
    }
    plot( target_tp,
          target_tpv,
          interp_path_tpv,
          output_path,
          path_index );
  }

  /// plot graph
  /// @param[in] target_tp       plotting target time-position data
  ///                            (ts, target_tp_s),
  ///                            (t1, target_tp_1),
  ///                            ... ,
  ///                            (tf, target_tp_f)
  /// @param[in] target_tpv      plotting target time-position-velocity data
  ///                            (ts, target_tpv_s),
  ///                            (t1, target_tpv_1),
  ///                            ... ,
  ///                            (tf, target_tpv_f)
  /// @param[in] interp_path_tpv plotting interpolated path
  ///                            of time-position-velocity with cycletime dT.
  ///                            (ts,     path(s)),
  ///                            (ts+dT,  path(ts+dT)),
  ///                            (ts+2dT, path(ts+2dT)),
  ///                            ... ,
  ///                            (tf,     path(tf))
  /// @param[in] output_path     destination path of plotted graph image(.png)
  /// @param[in] path_index      prefix index of destination path of plotted graph
  /// @details generate path of each set
  /// plot graph of time-position, time-velocity, position-velocity \n
  void plot( const TPQueue& target_tp,
             const TPVQueue& target_tpv,
             const TPVQueue& interp_path_tpv,
             const std::string& output_path="./",
             const int path_index = 0 ) {
    // target plot option
    plot_options_t options_target;
    // target plot type
    options_target["with"] = "point";
    // target point size
    options_target["ps"] = "5.0";
    // target point type
    options_target["pt"] = "3";
    // target line color
    options_target["lc"] = "rgb 'red'";
    // target point plot data (ts, t1, t2, ... , tf)
    // (ts, target_s), (t1, target_1), ... , (tf, target_f)
    std::vector<std::pair<double, double> >target_plot_data_tp;
    std::vector<std::pair<double, double> >target_plot_data_tv;
    std::vector<std::pair<double, double> >target_plot_data_phase;

    // interpolated plot options
    plot_options_t options;
    // interpolated plot type
    options["with"] = "lp";
    // interpolated point plot data (ts, ts+dT, ts+2dT, ... , tf)
    // (ts, path(s)), (ts+dT, path(ts+dT)), ... , (tf, path(tf))
    std::vector<std::pair<double, double> >path_plot_data_tp;
    std::vector<std::pair<double, double> >path_plot_data_tv;
    std::vector<std::pair<double, double> >path_plot_data_phase;

    // gnuplot
    GnuplotServer gpserver;

    // plot-data(time-target, time-interpolated-path)
    container_t multi_data;
    container_t multi_data_v;
    container_t multi_data_phase;
    // one set of time-posisiton-velocity
    TimePosition output;

    // input character into the graph
    std::stringstream numstr;
    // prepare output data log
    // set 2 digit lefty embeded 0
    numstr << output_path
           << std::setfill('0') << std::setw(2)
           << path_index << "_time_position_velocity.csv";
    std::ofstream ofstrm( numstr.str().c_str(), std::ios::out );
    numstr.str("");
    if (ofstrm.fail()) {
      std::cerr << "cannot open output temporary file."<< std::endl;
      FAIL();
    }
    // fix digit
    ofstrm << std::fixed << std::setprecision(8);

    for(std::size_t i=0; i < interp_path_tpv.size(); i++) {
      /////////////////////////////////////////////////////////////////////////
      /// output time-position-velocity data into the text file
      /////////////////////////////////////////////////////////////////////////
      ofstrm << interp_path_tpv.get(i).time
             << "," << interp_path_tpv.get(i).position
             << "," << interp_path_tpv.get(i).velocity << std::endl;
    }
    ofstrm.close();

    // add target of time-position into plotting buffer
    for(std::size_t i=0; i<target_tp.size(); i++) {
      target_plot_data_tp.push_back(
                            std::make_pair(target_tp.get(i).time,
                                           target_tp.get(i).position) );
    }
    for(std::size_t i=0; i<target_tpv.size(); i++) {
      // add target of time-velocity into plotting buffer
      target_plot_data_tv.push_back(
                            std::make_pair(target_tpv.get(i).time,
                                           target_tpv.get(i).velocity) );
      // add position-velocity (phase diagram) into plotting buffer
      target_plot_data_phase.push_back(
                               std::make_pair(target_tpv.get(i).position,
                                              target_tpv.get(i).velocity) );
    }
    // add interpolated_path of time-position, time-velocity, position-velocity into plotting buffer
    for(std::size_t i=0; i<interp_path_tpv.size(); i++) {
      path_plot_data_tp.push_back(
                          std::make_pair(interp_path_tpv.get(i).time,
                                         interp_path_tpv.get(i).position) );
      path_plot_data_tv.push_back(
                          std::make_pair(interp_path_tpv.get(i).time,
                                         interp_path_tpv.get(i).velocity) );
      path_plot_data_phase.push_back(
                             std::make_pair(interp_path_tpv.get(i).position,
                                            interp_path_tpv.get(i).velocity) );
    }
    // make final plot data by merging target & interpolated points
    // final plot data of time-position
    multi_data.push_back( std::make_pair(target_plot_data_tp,
                                         options_target) );
    multi_data.push_back( std::make_pair(path_plot_data_tp,
                                         options) );
    // final plot data of time-velocity
    multi_data_v.push_back( std::make_pair(target_plot_data_tv,
                                           options_target) );
    multi_data_v.push_back( std::make_pair(path_plot_data_tv,
                                           options) );
    // final plot data of position-velocity(phase diagram)
    multi_data_phase.push_back( std::make_pair(target_plot_data_phase,
                                               options_target) );
    multi_data_phase.push_back( std::make_pair(path_plot_data_phase,
                                               options) );

    /////////////////////////////////////////////////////////////////////////
    /// time-position plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2) << path_index << "_time-position_graph.png'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
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
    target_plot_data_tp.clear();
    path_plot_data_tp.clear();
    multi_data.clear();

    /////////////////////////////////////////////////////////////////////////
    /// time-velocity plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2)
           << path_index << "_time-velocity_graph.png'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
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
    target_plot_data_tv.clear();
    path_plot_data_tv.clear();
    multi_data_v.clear();

    /////////////////////////////////////////////////////////////////////////
    /// position-velocity plot
    /////////////////////////////////////////////////////////////////////////
    // set 2 digit lefty embeded 0
    numstr << "'" << output_path
           << std::setfill('0') << std::setw(2)
           << path_index << "_phase_diagram.png'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
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
    target_plot_data_phase.clear();
    path_plot_data_phase.clear();
    multi_data_phase.clear();

    usleep(0.01*1e6);
  }
};

} // end of namespace interp

#endif // INCLUDE_TEST_GRAPH_PLOT_HPP_
