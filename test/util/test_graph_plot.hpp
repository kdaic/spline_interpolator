#ifndef INCLUDE_TEST_GRAPH_PLOT_HPP_
#define INCLUDE_TEST_GRAPH_PLOT_HPP_

#include "spline_interpolator.hpp"
#include "test/util/gnuplot_realtime.hpp"

#include <fstream>

namespace interp {

/// graph plot class
class TestGraphPlot {
public:
  /// constructor
  TestGraphPlot() {
  };

  /// destructor
  ~TestGraphPlot() {};

  /// dump csv as "{path_index}_time_position_velocity.csv"
  /// @param[in] interp_path_tpva plotting interpolated path
  ///                             of time-position-velocity-acceleration with cycletime dT.
  ///                             (ts,     path(s)),
  ///                             (ts+dT,  path(ts+dT)),
  ///                             (ts+2dT, path(ts+2dT)),
  ///                             ... ,
  ///                             (tf,     path(tf))
  /// @param[in] output_dir       destination directory of plotted graph image(.png)
  /// @param[in] path_index       prefix index of destination path of plotted graph
  void dump_csv( const TPVAQueue& interp_path_tpva,
                 const std::string& output_dir="./",
                 const int path_index = 0 ) {

    // input character into the graph
    std::stringstream numstr;

    // prepare output data log
    // set 2 digit lefty embeded 0
    numstr << output_dir
           << std::setfill('0')
           << std::setw(4)
           << path_index << "_time_position_velocity.csv";

    std::ofstream ofstrm( numstr.str().c_str(), std::ios::out );
    if (ofstrm.fail()) {
      std::cerr << "cannot open output temporary file."<< std::endl;
      FAIL();
    }

    // fix digit
    ofstrm << std::fixed << std::setprecision(8);

    /////////////////////////////////////////////////////////////////////////
    /// output time-position-velocity data into the text file
    /////////////////////////////////////////////////////////////////////////
    for(std::size_t i=0; i < interp_path_tpva.size(); i++) {
      ofstrm << interp_path_tpva.get(i).time      << ","
             << interp_path_tpva.get(i).value.pos << ","
             << interp_path_tpva.get(i).value.vel << std::endl;
    }

    numstr.str("");
    ofstrm.close();
  };

  /// plot graph (overload)
  /// @param[in] target_tp        plotting target time-position data
  ///                             (ts, target_tp_s),
  ///                             (t1, target_tp_1),
  ///                             ... ,
  ///                             (tf, target_tp_f)
  /// @param[in] interp_path_tpva plotting interpolated path
  ///                             of time-position-velocity-acceleration with cycletime dT.
  ///                             (ts,     path(s)),
  ///                             (ts+dT,  path(ts+dT)),
  ///                             (ts+2dT, path(ts+2dT)),
  ///                             ... ,
  ///                             (tf,     path(tf))
  /// @param[in] output_dir       destination path of plotted graph image(.png)
  /// @param[in] path_index       prefix index of destination path of plotted graph
  void plot( const TPQueue& target_tp,
             const TPVAQueue& interp_path_tpva,
             const std::string& output_dir="./",
             const int path_index = 0 ) {

    TPVAQueue target_tpva;
    for(std::size_t i=0; i<target_tp.size(); i++) {
      target_tpva.push( target_tp.get(i).time,
                        PosVelAcc( target_tp.get(i).value ) );
    }
    plot( target_tpva,
          interp_path_tpva,
          output_dir,
          path_index,
          false );
  }

  /// plot graph
  /// @param[in] target_tpva      plotting target time-position-velocity data
  ///                             (ts, target_tpva_s),
  ///                             (t1, target_tpva_1),
  ///                              ... ,
  ///                             (tf, target_tpva_f)
  /// @param[in] interp_path_tpva plotting interpolated path
  ///                             of time-position-velocity with cycletime dT.
  ///                             (ts,     path(s)),
  ///                             (ts+dT,  path(ts+dT)),
  ///                             (ts+2dT, path(ts+2dT)),
  ///                             ... ,
  ///                             (tf,     path(tf))
  /// @param[in] output_dir       destination path of plotted graph image(.png)
  /// @param[in] path_index       prefix index of destination path of plotted graph
  /// @param[in] is_target_vel    wether target velocity data exists. default: true(exists)
  /// @details generate path of each set
  /// plot graph of time-position, time-velocity, position-velocity \n
  void plot( const TPVAQueue& target_tpva,
             const TPVAQueue& interp_path_tpva,
             const std::string& output_dir="./",
             const int path_index = 0,
             bool  is_target_vel=true ) {

    // target point plot data (ts, t1, t2, ... , tf)
    // (ts, target_s),
    // (t1, target_1),
    //  ... ,
    // (tf, target_f)
    std::vector<std::pair<double, double> >target_plot_data_tp;
    std::vector<std::pair<double, double> >target_plot_data_tv;
    std::vector<std::pair<double, double> >target_plot_data_pv;

    // interpolated point plot data (ts, ts+dT, ts+2dT, ... , tf)
    // (ts, path(s)),
    // (ts+dT, path(ts+dT)),
    //  ... ,
    // (tf, path(tf))
    std::vector<std::pair<double, double> >path_plot_data_tp;
    std::vector<std::pair<double, double> >path_plot_data_tv;
    std::vector<std::pair<double, double> >path_plot_data_pv;

    for(std::size_t i=0; i<target_tpva.size(); i++) {
      // add target of time-position into plotting buffer
      target_plot_data_tp.push_back(
                            std::make_pair( target_tpva.get(i).time,
                                            target_tpva.get(i).value.pos ) );
      if( is_target_vel ) {
        // add target of time-velocity into plotting buffer
        target_plot_data_tv.push_back(
                              std::make_pair( target_tpva.get(i).time,
                                              target_tpva.get(i).value.vel ) );
        // add position-velocity (phase diagram) into plotting buffer
        target_plot_data_pv.push_back(
                              std::make_pair( target_tpva.get(i).value.pos,
                                              target_tpva.get(i).value.vel ) );
      }
    }

    // add interpolated_path of
    //     time-position,
    //     time-velocity,
    //     position-velocity into plotting buffer
    for(std::size_t i=0; i<interp_path_tpva.size(); i++) {
      path_plot_data_tp.push_back(
                         std::make_pair( interp_path_tpva.get(i).time,
                                         interp_path_tpva.get(i).value.pos ) );
      path_plot_data_tv.push_back(
                          std::make_pair( interp_path_tpva.get(i).time,
                                          interp_path_tpva.get(i).value.vel ) );
      path_plot_data_pv.push_back(
                          std::make_pair( interp_path_tpva.get(i).value.pos,
                                          interp_path_tpva.get(i).value.vel ) );
    }

    /////////////////////////////////////////////////////////////////////////
    // prepare plotting
    /////////////////////////////////////////////////////////////////////////

    /// target plot option
    plot_options_t options_target;
    // target plot type
    options_target["with"] = "point";
    // target point size
    options_target["ps"] = "5.0";
    // target point type
    options_target["pt"] = "3";
    // target line color
    options_target["lc"] = "rgb 'red'";

    /// interpolated plot options
    plot_options_t options;
    // interpolated plot type
    options["with"] = "lp";

    // make final plot data by merging target & interpolated points.
    // plot-data includes time-target, time-interpolated-path.

    // final plot data of time-position
    container_t multi_data_tp;
    multi_data_tp.push_back( std::make_pair( target_plot_data_tp,
                                             options_target ) );
    multi_data_tp.push_back( std::make_pair( path_plot_data_tp,
                                             options ) );
    // final plot data of time-velocity
    container_t multi_data_tv;
    multi_data_tv.push_back( std::make_pair( target_plot_data_tv,
                                             options_target ) );
    multi_data_tv.push_back( std::make_pair( path_plot_data_tv,
                                             options ) );
    // final plot data of position-velocity(phase diagram)
    container_t multi_data_pv;
    multi_data_pv.push_back( std::make_pair( target_plot_data_pv,
                                             options_target ) );
    multi_data_pv.push_back( std::make_pair( path_plot_data_pv,
                                             options ) );

    // input character into the graph
    std::stringstream numstr;

    /////////////////////////////////////////////////////////////////////////
    /// time-position plot
    /////////////////////////////////////////////////////////////////////////

    // set 2 digit lefty embeded 0
    numstr << "'"
           << output_dir
           << std::setfill('0')
           << std::setw(4)
           << path_index << "_time-position_graph.png"
           << "'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
    gpserver_.set("output " +  numstr.str());
    numstr.str("");
    gpserver_.set("style data lp");
    gpserver_.set("xzeroaxis");
    gpserver_.set("grid");
    // gpserver_.set("noautoscale");
    gpserver_.set("xlabel 'time'");
    gpserver_.set("ylabel 'position'");
    gpserver_.set("nokey");
    gpserver_.flush();
    // plot
    gpserver_.plot(multi_data_tp);
    gpserver_.flush();
    // clear
    target_plot_data_tp.clear();
    path_plot_data_tp.clear();
    multi_data_tp.clear();

    /////////////////////////////////////////////////////////////////////////
    /// time-velocity plot
    /////////////////////////////////////////////////////////////////////////

    // set 2 digit lefty embeded 0
    numstr << "'"
           << output_dir
           << std::setfill('0')
           << std::setw(4)
           << path_index << "_time-velocity_graph.png"
           << "'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
    gpserver_.set("output " +  numstr.str());
    numstr.str("");
    gpserver_.set("style data lp");
    gpserver_.set("xzeroaxis");
    gpserver_.set("grid");
    // gpserver_.set("noautoscale");
    gpserver_.set("xlabel 'time'");
    gpserver_.set("ylabel 'velocity'");
    gpserver_.set("nokey");
    gpserver_.flush();
    // plot
    gpserver_.plot(multi_data_tv);
    gpserver_.flush();
    // clear
    target_plot_data_tv.clear();
    path_plot_data_tv.clear();
    multi_data_tv.clear();

    /////////////////////////////////////////////////////////////////////////
    /// position-velocity plot
    /////////////////////////////////////////////////////////////////////////

    // set 2 digit lefty embeded 0
    numstr << "'"
           << output_dir
           << std::setfill('0')
           << std::setw(4)
           << path_index << "_position-velocity_diagram.png"
           << "'";
    std::cout << "drawing " << numstr.str() << "..." << std::endl;
    gpserver_.set("output " +  numstr.str());
    numstr.str("");
    gpserver_.set("style data lp");
    gpserver_.set("xzeroaxis");
    gpserver_.set("grid");
    // gpserver_.set("noautoscale");
    gpserver_.set("xlabel 'position'");
    gpserver_.set("ylabel 'velocity'");
    gpserver_.set("nokey");
    gpserver_.flush();
    // plot
    gpserver_.plot(multi_data_pv);
    gpserver_.flush();
    // clear
    target_plot_data_pv.clear();
    path_plot_data_pv.clear();
    multi_data_pv.clear();

    // sleep 10[ms]
    usleep(0.01*1e6);
  }

  // gnuplot
  GnuplotServer gpserver_;
};

} // end of namespace interp

#endif // INCLUDE_TEST_GRAPH_PLOT_HPP_
