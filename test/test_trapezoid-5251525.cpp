#include "trapezoid_5251525.hpp"
#include "non_uniform_rounding_spline.hpp"
#include "test/util/gnuplot_realtime.hpp"
#include <deque>
#include <ctime>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include <gtest/gtest.h>

using namespace interp;

// 補間周期 0.05[s](=50[ms])
#define CYCLE 0.05
// 入力点周期 0.5[s]
#define PATH_CYCLE 0.5
// 入力点の数
#define POINT_NUM 100
/// 乱数パラメータ
#define MU 0.0
#define SIGMA 8.0
/// テスト開始位置
#define INIT_POSITION 0.0

static bool init_flag = false; ///< 乱数初期化フラグ

/// double型で乱数を生成
/// @return 乱数
/// @see rand()
double drand(double N){
  if(!init_flag) {
    srand((unsigned int)time(NULL));
    init_flag = true;
  }
  return rand() / ((double)RAND_MAX + 1) * N;
};

/// 正規化ノイズを発生させます。
/// Box-Muller法を採用しています。
/// @param mu 平均
/// @param sigma 偏差
/// @return ノイズ
/// @see <a href="http://ja.wikipedia.org/wiki/%E4%B9%B1%E6%95%B0">
///   Wikipedia『乱数列』</a>の正規乱数の項を参照のこと
double rand_regularized(double mu, double sigma){
  static double v_1, v_2, s;
  do{
    v_1 = drand(1.0) * 2 - 1;
    v_2 = drand(1.0) * 2 - 1;
  }while((s = v_1 * v_1 + v_2 * v_2) >= 1.0 || s == 0);
  return sigma * v_1 * sqrt(-2 * log(s) / s) + mu;
};

/////////////////////////////////////////////////////////////////////////////////////

/// 開始点から終端点(Point to Point)への軌道グラフ＆データファイルをプロットするクラス @n
/// 最大加速リミット、最大速度リミット、丸め率を設定し @n
/// 開始と終了の位置-時間-速度のセットを送ると、
/// 各セットのグラフ(.png)とデータ(.csv)を出力する @n
/// グラフは時間-位置, 時間-速度, 位置-速度(相図)の３種類 @n
class PlotPtoPGraph {
public:
  /// コンストラクタ
  /// @param output_path グラフ＆データファイルの出力先パス
  /// @param cycle プロット周期(デフォルト:0.05)
  /// @param acc_limit 最大加速度リミット(デフォルト:1200)
  /// @param vel_limit 最大速度リミット(デフォルト:170)
  /// @param smoothing_rate 丸め率(デフォルト:0.8)
  PlotPtoPGraph(const std::string& output_path="./",
                const double cycle=0.05,
                const double acc_limit=1200,
                const double vel_limit=170,
                const double smoothing_rate=0.8) :
    output_path_(output_path), cycle_(cycle),
    acc_limit_(acc_limit), vel_limit_(vel_limit),
    smoothing_rate_(smoothing_rate) {};

  /// デストラクタ
  ~PlotPtoPGraph(){};

  /// 軌道生成＆グラフプロット
  /// @param start_set 開始点の位置-時間-速度のセット
  /// @param goalgoal._set 終端点の位置-時間-速度のセット
  /// @details 各セットの軌道を生成し、
  /// 時間-位置,時間-速度,位置-速度のグラフを出力 @n
  void plot(const std::vector<TimePVA>& start_set,
            const std::vector<TimePVA>& goal_set) {
    // 描画入力目標点のバッファ
    std::deque<std::pair<double, double> > target;
    std::deque<std::pair<double, double> > target_v;
    std::deque<std::pair<double, double> > target_phase;
    // 描画軌道補間点のバッファ
    std::deque<std::pair<double, double> > path;
    std::deque<std::pair<double, double> > path_v;
    std::deque<std::pair<double, double> > path_phase;
    // 入力目標点の描画オプション
    plot_options_t options_target;
    options_target["with"] = "point";
    options_target["ps"] = "5.0";
    plot_options_t options;
    options["with"] = "lp";
    // 目標値描画データ
    std::vector<std::pair<double, double> >target_plot_data;
    std::vector<std::pair<double, double> >target_plot_data_v;
    std::vector<std::pair<double, double> >target_plot_data_phase;
    // 補間点描画データ
    std::vector<std::pair<double, double> >path_plot_data;
    std::vector<std::pair<double, double> >path_plot_data_v;
    std::vector<std::pair<double, double> >path_plot_data_phase;
    // 入力文字列
    std::stringstream numstr;
    // gnuplot
    GnuplotServer gpserver;
    // 描画点データ(時間に対する目標点列、時間に対する軌道補間点列)
    container_t multi_data;
    container_t multi_data_v;
    container_t multi_data_phase;
    // 軌道１セット分の開始点
    TimePVA start;
    // 軌道１セット分の目標到達点
    TimePVA goal;
    /// 各軌道セットについて軌道生成＆出力
    for(unsigned int index=0; index < start_set.size(); index++) {
      start = start_set[index];
      goal = goal_set[index];
      // 軌道生成
      Trapezoid5251525 tg(acc_limit_, acc_limit_,
                          vel_limit_,
                          smoothing_rate_, smoothing_rate_);
      double dT_total = tg.generate_path(start.time,  goal.time,
                                         start.P.pos, goal.P.pos,
                                         start.P.vel, goal.P.vel );
      std::cout << "dT_total:" << dT_total;
      double xt, vt, at;

      target.push_back( std::make_pair(start.time,            start.P.pos) );
      target.push_back( std::make_pair(start.time + dT_total, goal.P.pos) );

      target_v.push_back( std::make_pair(start.time,            start.P.vel) );
      target_v.push_back( std::make_pair(start.time + dT_total, goal.P.vel) );

      target_phase.push_back( std::make_pair(start.P.pos, start.P.vel) );
      target_phase.push_back( std::make_pair(goal.P.pos,  goal.P.vel) );

      // データログ出力用意
      // 左0詰め2桁設定
      numstr << "images/trapezoid-5251525/reachable/"
             << std::setfill('0') << std::setw(2) << index << "_time_position_velocity.csv";
      // ディレクトリ生成
      FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable", "re");
      pclose(mkdir);
      std::fstream cfstrm(numstr.str().c_str(), std::ios::out);
      numstr.str("");
      if (cfstrm.fail()) {
        std::cerr << "cannot open output temporary file."<< std::endl;
        FAIL();
      }
      // ログ桁固定
      cfstrm << std::fixed << std::setprecision(8);

      // プロットデータにキュー
      for(double t=start.time; t < tg.finish_time(); t+=cycle_) {
        tg.pop(t, xt, vt, at);
        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置-速度データファイル出力
        /////////////////////////////////////////////////////////////////////////
        {
          cfstrm << t << "," << xt << "," << vt << std::endl;
        }
        // 描画軌道補間点のバッファ追加
        path.push_back(std::make_pair(t, xt));
        path_v.push_back(std::make_pair(t, vt));
        path_phase.push_back(std::make_pair(xt, vt));
      }
      // 最後に目標到達点データを加える
      tg.pop(tg.finish_time(), xt, vt, at);
      {
        cfstrm << tg.finish_time() << "," << xt << "," << vt << std::endl;
      }
      cfstrm.close();
      path.push_back(std::make_pair(tg.finish_time(), xt));
      path_v.push_back(std::make_pair(tg.finish_time(), vt));
      path_phase.push_back(std::make_pair(xt, vt));

      // 時間-位置描画のバッファ追加
      for(std::deque<std::pair<double, double> >::iterator it = target.begin();
          it < target.end();
          it++) {
        target_plot_data.push_back(*it);
      }
      for(std::deque<std::pair<double, double> >::iterator it = path.begin();
          it < path.end();
          it++) {
        path_plot_data.push_back(*it);
      }
      multi_data.push_back(std::make_pair(target_plot_data, options_target));
      multi_data.push_back(std::make_pair(path_plot_data, options));

      // 時間-速度描画のバッファ追加
      for(std::deque<std::pair<double, double> >::iterator it = target_v.begin();
          it < target_v.end();
          it++) {
        target_plot_data_v.push_back(*it);
      }
      for(std::deque<std::pair<double, double> >::iterator it = path_v.begin();
          it < path_v.end();
          it++) {
        path_plot_data_v.push_back(*it);
      }
      multi_data_v.push_back(std::make_pair(target_plot_data_v, options_target));
      multi_data_v.push_back(std::make_pair(path_plot_data_v, options));

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
      multi_data_phase.push_back(std::make_pair(target_plot_data_phase, options_target));
      multi_data_phase.push_back(std::make_pair(path_plot_data_phase, options));

      /////////////////////////////////////////////////////////////////////////
      /// 時間-位置プロット
      /////////////////////////////////////////////////////////////////////////
      // 左0詰め2桁設定
      numstr << "'" << output_path_
             << std::setfill('0') << std::setw(2) << index << "_time-position_graph.png'";
      std::cerr << "drawing " << numstr.str() << "..." << std::endl;
      gpserver.set("output " +  numstr.str());
      numstr.str("");
      gpserver.set("style data lp");
      gpserver.set("xzeroaxis");
      // gpserver.set("noautoscale");
      gpserver.set("xlabel 'time'");
      gpserver.set("ylabel 'position'");
      gpserver.set("nokey");
      gpserver.flush();
      // プロット
      gpserver.plot(multi_data);
      gpserver.flush();
      // クリア
      target.clear();
      path.clear();
      target_plot_data.clear();
      path_plot_data.clear();
      multi_data.clear();

      /////////////////////////////////////////////////////////////////////////
      /// 時間-速度プロット
      /////////////////////////////////////////////////////////////////////////
      // 左0詰め2桁設定
      numstr << "'" << output_path_
             << std::setfill('0') << std::setw(2) << index << "_time-velocity_graph.png'";
      std::cerr << "drawing " << numstr.str() << "..." << std::endl;
      gpserver.set("output " +  numstr.str());
      numstr.str("");
      gpserver.set("style data lp");
      gpserver.set("xzeroaxis");
      // gpserver.set("noautoscale");
      gpserver.set("xlabel 'time'");
      gpserver.set("ylabel 'velocity'");
      gpserver.set("nokey");
      gpserver.flush();
      // プロット
      gpserver.plot(multi_data_v);
      gpserver.flush();
      // クリア
      target_v.clear();
      path_v.clear();
      target_plot_data_v.clear();
      path_plot_data_v.clear();
      multi_data_v.clear();

      /////////////////////////////////////////////////////////////////////////
      /// 位置-速度プロット
      /////////////////////////////////////////////////////////////////////////
      // 左0詰め2桁設定
      numstr << "'" << output_path_
             << std::setfill('0') << std::setw(2) << index << "_phase_diagram.png'";
      std::cerr << "drawing " << numstr.str() << "..." << std::endl;
      gpserver.set("output " +  numstr.str());
      numstr.str("");
      gpserver.set("style data lp");
      gpserver.set("xzeroaxis");
      // gpserver.set("noautoscale");
      gpserver.set("xlabel 'position'");
      gpserver.set("ylabel 'velocity'");
      gpserver.set("nokey");
      gpserver.flush();
      // プロット
      gpserver.plot(multi_data_phase);
      gpserver.flush();
      // クリア
      target_phase.clear();
      path_phase.clear();
      target_plot_data_phase.clear();
      path_plot_data_phase.clear();
      multi_data_phase.clear();
    }
    usleep(0.01*1e6);
  };

  /// グラフ＆データファイル出力先パス
  const std::string output_path_;
  /// プロット周期
  const double cycle_;
  /// 最大加速リミット
  const double acc_limit_;
  /// 最大速度リミット
  const double vel_limit_;
  /// 丸め率
  const double smoothing_rate_;
};

/////////////////////////////////////////////////////////////////////////////////////
/// @test 入力チェックのテスト @n
/// 同じ位置＆速度0が入ると初期位置、初期速度を返すことを確認
TEST(TrackingTest, same_start_and_goal) {
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 0.
  start.time  = 0.0;
  start.P.pos = 0;
  start.P.vel = 0.0;
  goal.time  = 0.5;
  goal.P.pos = 0;
  goal.P.vel = 0.0;
  Trapezoid5251525 tg;
  tg.generate_path(start.time,  goal.time,
                   start.P.pos, goal.P.pos,
                   start.P.vel, goal.P.vel);
  double xt, vt, at;
  for(double time=0.0; time <= goal.time; time+=0.1) {
    tg.pop(time, xt, vt, at);
    std::cout << "time: " << time;
    EXPECT_NEAR(start.P.pos, xt, 1e-15);
    EXPECT_NEAR(start.P.vel, vt, 1e-15);
  }
}

/// @test 到達不可能な指定時間 @n
/// 100%最速軌道で到達可能な時間より早い指定時間だとエラーを返すことを確認 @n
TEST(TrackingTest, time_error) {
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  start.time = 0.0;
  start.P.pos = 0;
  start.P.vel = 0.0;
  goal.time  = 0.001;
  goal.P.pos = 10;
  goal.P.vel = 100.0;
  Trapezoid5251525 tg;
  EXPECT_THROW(tg.generate_path(start.time,  goal.time,
                                start.P.pos, goal.P.pos,
                                start.P.vel, goal.P.vel),
               std::invalid_argument);
}

/// @test 到達限界テスト @n
/// 速度100%で到達できる範囲へ正常に制御できているか確認
TEST(TrackingTest, reachable) {
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 0.
  start.time  = 0.0;
  start.P.pos = 0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.5;
  goal.P.pos = 0;
  goal.P.vel = 100.0;
  goal_set.push_back(goal);
  // 1.
  start.time  = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 10.0;
  start_set.push_back(start);
  goal.time  = 0.0;
  goal.P.pos = 0.0;
  goal.P.vel = 100.0;
  goal_set.push_back(goal);
  // 2.
  start.time = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 100.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = 0.0;
  goal.P.vel = 10.0;
  goal_set.push_back(goal);
  // 3.
  start.time = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = 10.0;
  goal.P.vel = -100.0;
  goal_set.push_back(goal);
  // 4.
  start.time = 0.0;
  start.P.pos = 100.0;
  start.P.vel = -100.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = 0.0;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 5.
  start.time = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 10.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = 5.0;
  goal.P.vel = 100.0;
  goal_set.push_back(goal);
  // 6.
  start.time = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 10.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = -10.0;
  goal.P.vel = -100.0;
  goal_set.push_back(goal);
  // 7.
  start.time = 0.0;
  start.P.pos = 1.12345678901234567;
  start.P.vel = 10.12345678901234567;
  start_set.push_back(start);
  goal.time = 2.0;
  goal.P.pos = 1.123456789012345;
  goal.P.vel = -10.123456789012345;
  goal_set.push_back(goal);

  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable", "re");
  pclose(mkdir);
  // グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable/", 0.001);

  // 既に存在するデータ&画像を削除
  FILE* rm_time_position_velocity_log = popen("rm images/trapezoid-5251525/reachable/*.csv", "re");
  pclose(rm_time_position_velocity_log);
  FILE* rm_images = popen("rm images/trapezoid-5251525/reachable/*.png", "re");
  pclose(rm_images);

  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);
}

/// @test 到達限界テスト２
/// 実際の関節角パラメータを使用し、
/// 到達できる範囲へ正常に制御できているか確認 @n
TEST(TrackingTest, reachable2) {
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 0.
  start.time = 0.0;
  start.P.pos = -0.144616;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = -0.254926;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 2. およそ2倍の時間
  start.time = 0.0;
  start.P.pos = -0.144616;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 0.32;
  goal.P.pos = -0.254926;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 3. およそ10倍の時間
  start.time = 0.0;
  start.P.pos = -0.144616;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 1.6;
  goal.P.pos = -0.254926;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);

  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable2", "re");
  pclose(mkdir);
  // 既に存在する画像を削除
  FILE* rm_images = popen("rm images/trapezoid-5251525/reachable2/*.png", "re");
  pclose(rm_images);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable2/",
                                0.01,
                                31.415926535897931,
                                3.1415926535897931,
                                0.8);
  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);
}

/// @test 到達限界テスト３
/// 実際の関節角パラメータを使用し、
/// 到達できる範囲へ正常に制御できているか確認 @n
TEST(TrackingTest, reachable3) {
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 0.
  start.time = 0.0;
  start.P.pos = 0.142818547;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 0.0;
  goal.P.pos = -0.4658;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 1. およそ2倍の時間
  start.time = 0.0;
  start.P.pos = 0.142818547;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 0.90;
  goal.P.pos = -0.4658;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 2. およそ10倍の時間
  start.time = 0.0;
  start.P.pos = 0.142818547;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time = 4.5;
  goal.P.pos = -0.4658;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);


  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable3", "re");
  pclose(mkdir);

  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm images/trapezoid-5251525/reachable3/*.png", "re");
  pclose(rm_time_position_images);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable3/",
                                0.01,
                                13.17723585,
                                6.44026494,
                                0.01);
  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);
}


/// @test L1領域で時間指定時の到達限界テスト
/// 実際の直交系の最大加速度・最大速度パラメータを使用し、
/// 到達できる範囲へ正常に制御できているか確認 @n
TEST(TrackingTest, reachable_L1) {
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 0.
  start.time  = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.868808964551598;
  goal.P.pos = 0.0;
  goal.P.vel = 0.211391674843979;
  goal_set.push_back(goal);
  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable_L1", "re");
  pclose(mkdir);
  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable_L1/",
                                0.01,
                                1.2,
                                1.0,
                                0.8);
  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);
}


/// @test 速度リミット @n
/// 速度リミットを超える指定時間を設定するとエラーを返すことを確認
/// - 移動x[m] = -0.25 -> -0.10
/// - 速度リミット0.25[m/s]
/// - 加速度リミット1.0[m/s^2]
TEST(TrackingTest, speed_limit) {
  // リミット
  double vel_limit = 0.25;
  double acc_limit = 1.0;
  // 丸め率
  double smoothing_rate = 0.8;
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  start.time  = 0.0;
  start.P.pos = -0.25;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.55;
  goal.P.pos = -0.10;
  goal.P.vel = 0.0;
  goal_set.push_back(goal);
  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  Trapezoid5251525 tg(acc_limit, acc_limit,
                      vel_limit,
                      smoothing_rate, smoothing_rate);
  // 軌道生成＆グラフ出力(エラー出力)
  EXPECT_ANY_THROW( tg.generate_path(start.time, goal.time,
                                     start.P.pos, goal.P.pos,
                                     start.P.vel, goal.P.vel) );
}


/////////////////////////////////////////////////////////////////////////////////////

/// @test ランダムプロット @n
/// 目標点は5点ずつ表示 @n
/// 軌跡は2点遅れて追跡(数は1/CYCLE倍) @n
/// 乱数で生成された入力目標位置の周期時間列を @n
/// 不均一丸みスプラインにより滑らかに速度接続し、 @n
/// 軌道生成の結果を連番画像で出力する @n
TEST(TrackingTest, random_plot) {
  // 描画入力目標点の数
  const unsigned int target_num = 5;
  // 描画軌道補間点の数
  const unsigned int path_num = static_cast<unsigned int>((target_num)*PATH_CYCLE/CYCLE);
  // 描画入力目標点のバッファ
  std::deque<std::pair<double, double> > target;
  std::deque<std::pair<double, double> > target_v;
  // 描画軌道補間点のバッファ
  std::deque<std::pair<double, double> > path;
  std::deque<std::pair<double, double> > path_v;
  // 入力目標点の描画オプション
  plot_options_t options_target;
  options_target["with"] = "point";
  options_target["ps"] = "5.0";
  plot_options_t options;
  options["with"] = "lp";
  // 目標値描画データ
  std::vector<std::pair<double, double> >target_plot_data;
  std::vector<std::pair<double, double> >target_plot_data_v;
  // 補間点描画データ
  std::vector<std::pair<double, double> >path_plot_data;
  std::vector<std::pair<double, double> >path_plot_data_v;
  // プロットコマンド入力文字列
  std::stringstream numstr;
  // gnuplot
  GnuplotServer gpserver;
  // 描画点データ(時間に対する目標点列、時間に対する軌道補間点列)
  container_t multi_data;
  container_t multi_data_v;

  // ディレクトリ作成
  FILE* mkdir_p = popen("mkdir -p images/trapezoid-5251525/random_plot/time_position", "re");
  pclose(mkdir_p);
  FILE* mkdir_v = popen("mkdir -p images/trapezoid-5251525/random_plot/time_velocity", "re");
  pclose(mkdir_v);

  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm images/trapezoid-5251525/random_plot/time_position/*.png", "re");
  pclose(rm_time_position_images);
  FILE* rm_time_velocity_images = popen("rm images/trapezoid-5251525/random_plot/time_velocity/*.png", "re");
  pclose(rm_time_velocity_images);

  // 目標位置(初期化=0.0)
  double target_position=0.0;
  // 時間・位置・速度のバッファ
  NonUniformRoundingSpline nusv(INIT_POSITION);
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  double start_time=0.0;

  // 目標点生成のサイクル
  int index = 0;
  for (int pindex=0; pindex < POINT_NUM; pindex++) {
    if(pindex == 0) {
      target_position = 0.0;
    } else {
      // 目標位置を乱数で生成
      target_position = rand_regularized(MU, SIGMA);
      // 目標位置から目標時間・位置・速度の点を生成(3点分貯める)
      nusv.push(PATH_CYCLE, target_position);
    }

    target.push_back( std::make_pair(pindex * PATH_CYCLE, target_position) );

    if(pindex == 2) {
      start = nusv.pop();
      target_v.push_back( std::make_pair((pindex-2) * PATH_CYCLE, start.P.vel) );
      continue;
    }

    if(nusv.size() > 2) {
      goal = nusv.pop();
      target_v.push_back( std::make_pair((pindex-2) * PATH_CYCLE, goal.P.vel) );

      // 軌道生成
      Trapezoid5251525 tg;
      tg.generate_path(start.time,  goal.time,
                       start.P.pos, goal.P.pos,
                       start.P.vel, goal.P.vel);
      double xt, vt, at;
      // プロットデータにキュー
      for(double t=start_time; t <= tg.finish_time()+1e-12; t+=CYCLE, index++) {
        tg.pop(t, xt, vt, at);
        if(path.size() > path_num) {
          path.pop_front();
        }
        if(path_v.size() > path_num) {
          path_v.pop_front();
        }
        // 描画軌道補間点のバッファ追加
        path.push_back(std::make_pair(t, xt));
        path_v.push_back(std::make_pair(t, vt));

        // 描画入力目標点のバッファ除去
        if(target.front().first < path.front().first) {
          target.pop_front();
        }
        if(target_v.front().first < path_v.front().first) {
          target_v.pop_front();
        }

        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置プロット
        /////////////////////////////////////////////////////////////////////////

        // 左0詰め4桁設定
        numstr << "'images/trapezoid-5251525/random_plot/time_position/graph_"
               << std::setfill('0') << std::setw(4) << index << ".png'";

        std::cerr << "drawing " << numstr.str() << "..." << std::endl;
        gpserver.set("output " +  numstr.str());
        numstr.str("");
        gpserver.set("style data lp");
        gpserver.set("xzeroaxis");
        // gpserver.set("noautoscale");
        gpserver.set("xlabel 'time'");
        gpserver.set("ylabel 'position'");
        numstr << path.front().first << ":" << (path.front().first + (target_num+4)*PATH_CYCLE) ;
        gpserver.set("xrange ["+ numstr.str() + "]");
        numstr.str("");
        gpserver.set("yrange [-30:30]");
        gpserver.set("nokey");
        gpserver.flush();

        for(std::deque<std::pair<double, double> >::iterator it = target.begin();
            it < target.end();
            it++) {
          target_plot_data.push_back(*it);
        }
        for(std::deque<std::pair<double, double> >::iterator it = path.begin();
            it < path.end();
            it++) {
          path_plot_data.push_back(*it);
        }
        multi_data.push_back(std::make_pair(target_plot_data, options_target));
        multi_data.push_back(std::make_pair(path_plot_data, options));

        gpserver.plot(multi_data);
        gpserver.flush();

        /////////////////////////////////////////////////////////////////////////
        /// 時間-速度プロット
        /////////////////////////////////////////////////////////////////////////

        // 左0詰め4桁設定
        numstr << "'images/trapezoid-5251525/random_plot/time_velocity/graph_"
               << std::setfill('0') << std::setw(4) << index << ".png'";
        std::cerr << "drawing " << numstr.str() << "..." << std::endl;
        gpserver.set("output " +  numstr.str());
        numstr.str("");
        gpserver.set("style data lp");
        gpserver.set("xzeroaxis");
        // gpserver.set("noautoscale");
        gpserver.set("xlabel 'time'");
        gpserver.set("ylabel 'velocity'");
        numstr << path_v.front().first << ":" << (path_v.front().first + (target_num+4)*PATH_CYCLE) ;
        gpserver.set("xrange ["+ numstr.str() + "]");
        numstr.str("");
        numstr << -tg.v_limit()*0.7 << ":" << tg.v_limit()*0.7 ;
        gpserver.set("yrange ["+ numstr.str() + "]");
        numstr.str("");
        gpserver.set("nokey");
        gpserver.flush();

        for(std::deque<std::pair<double, double> >::iterator it = target_v.begin();
            it < target_v.end();
            it++) {
          target_plot_data_v.push_back(*it);
        }
        for(std::deque<std::pair<double, double> >::iterator it = path_v.begin();
            it < path_v.end();
            it++) {
          path_plot_data_v.push_back(*it);
        }

        multi_data_v.push_back(std::make_pair(target_plot_data_v, options_target));
        multi_data_v.push_back(std::make_pair(path_plot_data_v, options));

        gpserver.plot(multi_data_v);
        gpserver.flush();

        target_plot_data.clear();
        path_plot_data.clear();
        multi_data.clear();

        target_plot_data_v.clear();
        path_plot_data_v.clear();
        multi_data_v.clear();
        usleep(0.01*1e6);
      } // end of プロットデータにキュー

      start_time = tg.finish_time();
      start = goal;

    } // end of if(nusv.size() >= 3)

  }// end of 目標点生成のサイクル
}

/////////////////////////////////////////////////////////////////////////////////////

/// @test x-y平面プロット @n
/// x-y平面で軌道生成 @n
/// 軌跡は2点遅れて追跡(数は1/CYCLE倍) @n
/// 四角形、鈍角(どんかく)で結ばれる経由点を目標入力とする @n
/// 各目標入力の指定時間は一定周期とする @n
/// 不均一丸みスプラインにより滑らかに速度接続し、 @n
/// 軌道生成の結果を連番画像で出力する @n
TEST(TrackingTest, plot_xy) {
  // 目標位置(正方形)
  const double target_position_x[] = {0.0,  0.0,  40.0, 40.0,
                                      20.0, 10.0, 10.0, 20.0,
                                      30.0, 35.0, 35.0, 30.0, 10.0, 10.0, 0};
  const double target_position_y[] = {0.0,  40.0, 40.0, 0.0,
                                      0.0,  10.0, 20.0, 30.0,
                                      30.0, 25.0, 20.0, 15.0, 15.0, 20.0, 0};
  // 移動幅
  const double range_x = 40.0;
  const double range_y = 40.0;
  // 描画入力目標点の数
  const unsigned int target_num = sizeof(target_position_x) / sizeof(target_position_x[0]);
  const unsigned int target_time_num = 5;
  // 描画軌道補間点の数
  const unsigned int path_num = static_cast<unsigned int>((target_num)*PATH_CYCLE/CYCLE);
  const unsigned int path_time_num = static_cast<unsigned int>((target_time_num)*PATH_CYCLE/CYCLE);
  // 描画入力目標点のバッファ
  std::deque<std::pair<double, double> > target;
  std::deque<std::pair<double, double> > target_x;
  std::deque<std::pair<double, double> > target_vx;
  std::deque<std::pair<double, double> > target_y;
  std::deque<std::pair<double, double> > target_vy;
  // 描画軌道補間点のバッファ
  std::deque<std::pair<double, double> > path;
  std::deque<std::pair<double, double> > path_x;
  std::deque<std::pair<double, double> > path_vx;
  std::deque<std::pair<double, double> > path_y;
  std::deque<std::pair<double, double> > path_vy;
  // 入力目標点の描画オプション
  plot_options_t options_target;
  options_target["with"] = "point";
  options_target["ps"] = "5.0";
  plot_options_t options;
  options["with"] = "lp";
  // 目標値描画データ
  std::vector<std::pair<double, double> >target_plot_data;
  std::vector<std::pair<double, double> >target_plot_data_x;
  std::vector<std::pair<double, double> >target_plot_data_vx;
  std::vector<std::pair<double, double> >target_plot_data_y;
  std::vector<std::pair<double, double> >target_plot_data_vy;
  // 補間点描画データ
  std::vector<std::pair<double, double> >path_plot_data;
  std::vector<std::pair<double, double> >path_plot_data_x;
  std::vector<std::pair<double, double> >path_plot_data_vx;
  std::vector<std::pair<double, double> >path_plot_data_y;
  std::vector<std::pair<double, double> >path_plot_data_vy;
  // プロットコマンド入力文字列
  std::stringstream numstr;
  // gnuplot
  GnuplotServer gpserver;
  // 描画点データ(時間に対する目標点列、時間に対する軌道補間点列)
  container_t multi_data;

  // 時間・位置・速度のバッファ
  NonUniformRoundingSpline nusv_x(INIT_POSITION);
  NonUniformRoundingSpline nusv_y(INIT_POSITION);
  // 軌道１セット分の開始点
  TimePVA start_x;
  TimePVA start_y;
  // 軌道１セット分の目標到達点
  TimePVA goal_x;
  TimePVA goal_y;
  double start_time=0.0;

  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/xy_plot", "re");
  pclose(mkdir);
  // ファイル出力
  std::fstream cfstrm("images/trapezoid-5251525/xy_plot/time_xy_position.csv", std::ios::out);
  if (cfstrm.fail()) {
    std::cerr << "cannot open output temporary file."<< std::endl;
    FAIL();
  }

  // ディレクトリ作成
  FILE* mkdir_p = popen("mkdir -p images/trapezoid-5251525/xy_plot/time_position", "re");
  pclose(mkdir_p);
  FILE* mkdir_v = popen("mkdir -p images/trapezoid-5251525/xy_plot/time_velocity", "re");
  pclose(mkdir_v);
  FILE* mkdir_xy = popen("mkdir -p images/trapezoid-5251525/xy_plot/xy", "re");
  pclose(mkdir_xy);

  // 桁固定
  cfstrm << std::fixed << std::setprecision(8);
  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm images/trapezoid-5251525/xy_plot/time_position/*.png", "re");
  pclose(rm_time_position_images);
  FILE* rm_time_velocity_images = popen("rm images/trapezoid-5251525/xy_plot/time_velocity/*.png", "re");
  pclose(rm_time_velocity_images);
  FILE* rm_xy_images = popen("rm images/trapezoid-5251525/xy_plot/xy/*.png", "re");
  pclose(rm_xy_images);

  // 目標点生成のサイクル
  int index = 0;
  for (unsigned int pindex=0; pindex < target_num; pindex++) {
    std::cout << "pindex : " << pindex << "/" << target_num-4;
    if(pindex > 0) {
      // 目標位置から目標時間・位置・速度の点を生成(3点分貯める)
      nusv_x.push(PATH_CYCLE, target_position_x[pindex]);
      nusv_y.push(PATH_CYCLE, target_position_y[pindex]);
    }

    target.push_back( std::make_pair(target_position_x[pindex], target_position_y[pindex]) );
    target_x.push_back( std::make_pair(pindex * PATH_CYCLE, target_position_x[pindex]) );
    target_y.push_back( std::make_pair(pindex * PATH_CYCLE, target_position_y[pindex]) );

    if(pindex == 2) {
      start_x = nusv_x.pop();
      start_y = nusv_y.pop();
      target_vx.push_back( std::make_pair((pindex-2) * PATH_CYCLE, start_x.P.vel) );
      target_vy.push_back( std::make_pair((pindex-2) * PATH_CYCLE, start_y.P.vel) );
      continue;
    }

    if(nusv_x.size() > 2) {
      goal_x = nusv_x.pop();
      goal_y = nusv_y.pop();
      target_vx.push_back( std::make_pair((pindex-2) * PATH_CYCLE, goal_x.P.vel) );
      target_vy.push_back( std::make_pair((pindex-2) * PATH_CYCLE, goal_y.P.vel) );

      // 軌道生成
      Trapezoid5251525 tg_x;
      tg_x.generate_path(start_x.time,  goal_x.time,
                         start_x.P.pos, goal_x.P.pos,
                         start_x.P.vel, goal_x.P.vel);
      Trapezoid5251525 tg_y;
      tg_y.generate_path(start_y.time,  goal_y.time,
                         start_y.P.pos, goal_y.P.pos,
                         start_y.P.vel, goal_y.P.vel);
      double xt, v_xt, a_xt;
      double yt, v_yt, a_yt;
      // プロットデータにキュー
      for(double t=start_time; t <= tg_x.finish_time()+1e-12; t+=CYCLE, index++) {
        tg_x.pop(t, xt, v_xt, a_xt);
        tg_y.pop(t, yt, v_yt, a_yt);
        if(path.size() > path_num) {
          path.pop_front();
        }
        if(path_x.size() > path_time_num) {
          path_x.pop_front();
          path_y.pop_front();
        }
        if(path_vx.size() > path_time_num) {
          path_vx.pop_front();
          path_vy.pop_front();
        }
        // 描画軌道補間点のバッファ追加
        path.push_back(std::make_pair(xt, yt));
        path_x.push_back(std::make_pair(t, xt));
        path_y.push_back(std::make_pair(t, yt));
        path_vx.push_back(std::make_pair(t, v_xt));
        path_vy.push_back(std::make_pair(t, v_yt));

        // 描画入力目標点のバッファ除去
        if(target.front().first < path.front().first) {
          target.pop_front();
        }
        if(target_x.front().first < path_x.front().first) {
          target_x.pop_front();
          target_y.pop_front();
        }
        if(target_vx.front().first < path_vx.front().first) {
          target_vx.pop_front();
          target_vy.pop_front();
        }

        /////////////////////////////////////////////////////////////////////////
        /// xy平面プロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/xy_plot/xy/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'X-position'");
          gpserver.set("ylabel 'Y-position'");
          numstr << target_position_x[0]-0.1*range_x << ":" << 1.1*range_x;
          gpserver.set("xrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr << target_position_y[0]-0.1*range_y << ":" << 1.1*range_y;
          gpserver.set("yrange ["+ numstr.str() + "]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();

          for(std::deque<std::pair<double, double> >::iterator it = target.begin();
              it < target.end();
              it++) {
            target_plot_data.push_back(*it);
          }
          for(std::deque<std::pair<double, double> >::iterator it = path.begin();
              it < path.end();
              it++) {
            path_plot_data.push_back(*it);
          }
          multi_data.push_back(std::make_pair(target_plot_data, options_target));
          multi_data.push_back(std::make_pair(path_plot_data, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data.clear();
          path_plot_data.clear();
          multi_data.clear();
          usleep(0.01*1e6);
        }

        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置x/y平面データファイル出力
        /////////////////////////////////////////////////////////////////////////
        {
          cfstrm << t << "," << xt << "," << yt << std::endl;
        }

        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置x/y平面プロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/xy_plot/time_position/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'time'");
          gpserver.set("ylabel 'X/Y-position'");
          numstr
            << path_x.front().first << ":" << (path_x.front().first + (target_time_num+4)*PATH_CYCLE) ;
          gpserver.set("xrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr << target_position_x[0]-0.1*range_x << ":" << 1.1*range_x;
          gpserver.set("yrange ["+ numstr.str() + "]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();

          std::deque<std::pair<double, double> >::iterator tg_it_x;
          std::deque<std::pair<double, double> >::iterator tg_it_y;
          for( tg_it_x = target_x.begin(), tg_it_y = target_y.begin();
               tg_it_x < target_x.end();
               tg_it_x++, tg_it_y++) {
            target_plot_data_x.push_back(*tg_it_x);
            target_plot_data_y.push_back(*tg_it_y);
          }
          std::deque<std::pair<double, double> >::iterator path_it_x;
          std::deque<std::pair<double, double> >::iterator path_it_y;
          for( path_it_x = path_x.begin(), path_it_y = path_y.begin();
               path_it_x < path_x.end();
               path_it_x++, path_it_y++) {
            path_plot_data_x.push_back(*path_it_x);
            path_plot_data_y.push_back(*path_it_y);
          }
          multi_data.push_back(std::make_pair(target_plot_data_x, options_target));
          multi_data.push_back(std::make_pair(target_plot_data_y, options_target));
          multi_data.push_back(std::make_pair(path_plot_data_x, options));
          multi_data.push_back(std::make_pair(path_plot_data_y, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data_x.clear();
          target_plot_data_y.clear();
          path_plot_data_x.clear();
          path_plot_data_y.clear();
          multi_data.clear();
        }
        usleep(0.01*1e6);

        /////////////////////////////////////////////////////////////////////////
        /// 時間-速度x/yプロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/xy_plot/time_velocity/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'time'");
          gpserver.set("ylabel 'X/Y-velocity'");
          numstr
            << path_vx.front().first << ":" << (path_vx.front().first + (target_time_num+4)*PATH_CYCLE) ;
          gpserver.set("xrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr << -tg_x.v_limit() << ":" << tg_x.v_limit();
          gpserver.set("yrange ["+ numstr.str() + "]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();
          std::deque<std::pair<double, double> >::iterator tg_it_vx;
          std::deque<std::pair<double, double> >::iterator tg_it_vy;
          for( tg_it_vx = target_vx.begin(), tg_it_vy = target_vy.begin();
               tg_it_vx < target_vx.end();
               tg_it_vx++, tg_it_vy++) {
            target_plot_data_vx.push_back(*tg_it_vx);
            target_plot_data_vy.push_back(*tg_it_vy);
          }
          std::deque<std::pair<double, double> >::iterator path_it_vx;
          std::deque<std::pair<double, double> >::iterator path_it_vy;
          for( path_it_vx = path_vx.begin(), path_it_vy = path_vy.begin();
               path_it_vx < path_vx.end();
               path_it_vx++, path_it_vy++ ) {
            path_plot_data_vx.push_back(*path_it_vx);
            path_plot_data_vy.push_back(*path_it_vy);
          }
          multi_data.push_back(std::make_pair(target_plot_data_vx, options_target));
          multi_data.push_back(std::make_pair(target_plot_data_vy, options_target));
          multi_data.push_back(std::make_pair(path_plot_data_vx, options));
          multi_data.push_back(std::make_pair(path_plot_data_vy, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data_vx.clear();
          target_plot_data_vy.clear();
          path_plot_data_vx.clear();
          path_plot_data_vy.clear();
          multi_data.clear();
        }
        usleep(0.01*1e6);
      } // end of プロットデータにキュー

      start_time = tg_x.finish_time();
      start_x = goal_x;
      start_y = goal_y;

    } // end of if(nusv_x.size() >= 3)

  }// end of 目標点生成のサイクル
  cfstrm.close();
}

/// @test データ入力 @n
/// 直交空間の目標入力データからxy軌道生成 @n
/// 軌跡は2点遅れて追跡(数は1/CYCLE倍) @n
/// 四角形で結ばれる経由点を目標入力とする @n
/// 各目標入力の指定時間は一定周期とする @n
/// 不均一丸みスプラインにより滑らかに速度接続し、 @n
/// 軌道生成の結果を連番画像で出力する @n
TEST(TrackingTest, datafile_xy) {
  // 直交系の最大加速、速度リミット
  const double acc_limit = 30;
  const double vel_limit = 2;
  // データファイル(csv形式)
  std::ifstream ifstrm("test/data/teaching_points_5p.csv");
  if (ifstrm.fail()) {
    std::cerr << "cannot open input temporary file."<< std::endl;
    FAIL();
  }
  // パース
  // データテーブル[行][列]
  std::vector< std::vector<double> > table_data;
  std::string line;
  while (std::getline(ifstrm, line)) {
    if(line.c_str()[0] == '#') continue; // 一行目はラベル
    std::istringstream isstrm(line);
    std::string element;
    std::vector<double> line_data;
    while (std::getline(isstrm, element, ',')) {
      line_data.push_back(atof(element.c_str()));
      std::cerr << element << ",";
    }
    std::cerr << std::endl;
    table_data.push_back(line_data);
  }
  ifstrm.close();
  // 描画入力目標点の生成
  const double target_time[] = {0.0,      0.504564, 0.219867,
                                0.544634, 0.289216, 0.296369,
                                0.0, 0.0};
  double last_time_clock = 0.0;
  for (unsigned int i=0; i< sizeof(target_time)/sizeof(target_time[0]); i++) {
    last_time_clock += target_time[i];
  }
  std::vector<double> target_position_x;
  std::vector<double> target_position_y;
  ///
  for (unsigned int i=0; i<table_data.size(); i++) {
    target_position_x.push_back(table_data[i][0]);
    target_position_y.push_back(table_data[i][1]);
    std::cerr << "[" << i << "] x, y :" << table_data[i][0] << ", " << table_data[i][1] << std::endl;
  }
  // 終了後のデータ
  target_position_x.push_back(target_position_x[0]);
  target_position_y.push_back(target_position_y[0]);

  // 描画入力目標点の数
  const unsigned int target_num = target_position_x.size();
  std::cerr << "target_num : " << target_num << std::endl;
  // 描画入力目標点のバッファ
  std::deque<std::pair<double, double> > target;
  std::deque<std::pair<double, double> > target_x;
  std::deque<std::pair<double, double> > target_vx;
  std::deque<std::pair<double, double> > target_y;
  std::deque<std::pair<double, double> > target_vy;
  // 描画軌道補間点のバッファ
  std::deque<std::pair<double, double> > path;
  std::deque<std::pair<double, double> > path_x;
  std::deque<std::pair<double, double> > path_vx;
  std::deque<std::pair<double, double> > path_y;
  std::deque<std::pair<double, double> > path_vy;
  // 入力目標点の描画オプション
  plot_options_t options_target;
  options_target["with"] = "point";
  options_target["ps"] = "5.0";
  plot_options_t options;
  options["with"] = "lp";
  // 目標値描画データ
  std::vector<std::pair<double, double> >target_plot_data;
  std::vector<std::pair<double, double> >target_plot_data_x;
  std::vector<std::pair<double, double> >target_plot_data_vx;
  std::vector<std::pair<double, double> >target_plot_data_y;
  std::vector<std::pair<double, double> >target_plot_data_vy;
  // 補間点描画データ
  std::vector<std::pair<double, double> >path_plot_data;
  std::vector<std::pair<double, double> >path_plot_data_x;
  std::vector<std::pair<double, double> >path_plot_data_vx;
  std::vector<std::pair<double, double> >path_plot_data_y;
  std::vector<std::pair<double, double> >path_plot_data_vy;
  // プロットコマンド入力文字列
  std::stringstream numstr;
  // gnuplot
  GnuplotServer gpserver;
  // 描画点データ(時間に対する目標点列、時間に対する軌道補間点列)
  container_t multi_data;

  // 時間・位置・速度のバッファ
  NonUniformRoundingSpline nusv_x(target_position_x[0]);
  NonUniformRoundingSpline nusv_y(target_position_y[0]);
  // 軌道１セット分の開始点
  TimePVA start_x;
  TimePVA start_y;
  // 軌道１セット分の目標到達点
  TimePVA goal_x;
  TimePVA goal_y;
  double start_time=0.0;

  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/datafile_xy", "re");
  pclose(mkdir);

  // ファイル出力
  std::fstream cfstrm("images/trapezoid-5251525/datafile_xy/time_xy_position.csv", std::ios::out);
  if (cfstrm.fail()) {
    std::cerr << "cannot open output temporary file."<< std::endl;
    FAIL();
  }

  // ディレクトリ作成
  FILE* mkdir_p = popen("mkdir -p images/trapezoid-5251525/datafile_xy/time_position", "re");
  pclose(mkdir_p);
  FILE* mkdir_v = popen("mkdir -p images/trapezoid-5251525/datafile_xy/time_velocity", "re");
  pclose(mkdir_v);
  FILE* mkdir_xy = popen("mkdir -p images/trapezoid-5251525/datafile_xy/xy", "re");
  pclose(mkdir_xy);
  
  // 桁固定
  cfstrm << std::fixed << std::setprecision(8);
  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm images/trapezoid-5251525/datafile_xy/time_position/*.png", "re");
  pclose(rm_time_position_images);
  FILE* rm_time_velocity_images = popen("rm images/trapezoid-5251525/datafile_xy/time_velocity/*.png", "re");
  pclose(rm_time_velocity_images);
  FILE* rm_xy_images = popen("rm images/trapezoid-5251525/datafile_xy/xy/*.png", "re");
  pclose(rm_xy_images);


  // 目標点生成のサイクル
  int index = 0;
  double timecount = target_time[0];
  for (unsigned int pindex=1; pindex <= target_num; pindex++) {
    timecount += target_time[pindex];
    std::cout << "pindex : " << pindex << "/" << target_num;
    if(pindex > 0) {
      // 目標位置から目標時間・位置・速度の点を生成(3点分貯める)
      nusv_x.push(target_time[pindex], target_position_x[pindex]);
      nusv_y.push(target_time[pindex], target_position_y[pindex]);
    }

    target.push_back( std::make_pair(target_position_x[pindex], target_position_y[pindex]) );
    target_x.push_back( std::make_pair(timecount,
                                       target_position_x[pindex]) );
    target_y.push_back( std::make_pair(timecount,
                                       target_position_y[pindex]) );

    if(pindex == 2) {
      start_x = nusv_x.pop();
      start_y = nusv_y.pop();
      target_vx.push_back( std::make_pair(start_x.time,
                                          start_x.P.vel) );
      target_vy.push_back( std::make_pair(start_y.time,
                                          start_y.P.vel) );
      continue;
    }

    if(nusv_x.size() > 2) {
      goal_x = nusv_x.pop();
      goal_y = nusv_y.pop();
      std::cerr << "goal_time :" << goal_x.time << std::endl;
      std::cerr << "interval_time : " << target_time[pindex-2] << std::endl;
      std::cerr << "goal_x :" << goal_x.P.pos << std::endl;
      std::cerr << "goal_y :" << goal_y.P.pos << std::endl;
      target_vx.push_back( std::make_pair(goal_x.time, goal_x.P.vel) );
      target_vy.push_back( std::make_pair(goal_x.time, goal_y.P.vel) );

      // 軌道生成
      Trapezoid5251525 tg_x(acc_limit, acc_limit, vel_limit);
      tg_x.generate_path(start_x.time,  goal_x.time,
                         start_x.P.pos, goal_x.P.pos,
                         start_x.P.vel, goal_x.P.vel);
      Trapezoid5251525 tg_y(acc_limit, acc_limit, vel_limit);
      tg_y.generate_path(start_y.time,  goal_y.time,
                         start_y.P.pos, goal_y.P.pos,
                         start_y.P.vel, goal_y.P.vel);
      // プロットデータにキュー
      double xt, v_xt, a_xt;
      double yt, v_yt, a_yt;
      double t;
      for(t=start_time; t <= tg_x.finish_time()+1e-12; t+=CYCLE, index++) {
        tg_x.pop(t, xt, v_xt, a_xt);
        tg_y.pop(t, yt, v_yt, a_yt);
        // 描画軌道補間点のバッファ追加
        path.push_back(std::make_pair(xt, yt));
        path_x.push_back(std::make_pair(t, xt));
        path_y.push_back(std::make_pair(t, yt));
        path_vx.push_back(std::make_pair(t, v_xt));
        path_vy.push_back(std::make_pair(t, v_yt));

        /////////////////////////////////////////////////////////////////////////
        /// xy平面プロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/datafile_xy/xy/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'X-position'");
          gpserver.set("ylabel 'Y-position'");
          gpserver.set("xrange [0.0:0.5]");
          gpserver.set("yrange [-0.5:0.1]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();

          for(std::deque<std::pair<double, double> >::iterator it = target.begin();
              it < target.end();
              it++) {
            target_plot_data.push_back(*it);
          }
          for(std::deque<std::pair<double, double> >::iterator it = path.begin();
              it < path.end();
              it++) {
            path_plot_data.push_back(*it);
          }
          multi_data.push_back(std::make_pair(target_plot_data, options_target));
          multi_data.push_back(std::make_pair(path_plot_data, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data.clear();
          path_plot_data.clear();
          multi_data.clear();
          usleep(0.01*1e6);
        }

        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置x/y平面データファイル出力
        /////////////////////////////////////////////////////////////////////////
        {
          cfstrm << t << "," << xt << "," << yt << std::endl;
        }

        /////////////////////////////////////////////////////////////////////////
        /// 時間-位置x/y平面プロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/datafile_xy/time_position/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'time'");
          gpserver.set("ylabel 'X/Y-position'");
          gpserver.set("xrange []");
          numstr << "0.0 :" << last_time_clock+0.1;
          gpserver.set("xrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr << -tg_x.v_limit() << ":" << tg_x.v_limit();
          gpserver.set("yrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr.str("");
          gpserver.set("yrange [-0.5:0.5]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();

          std::deque<std::pair<double, double> >::iterator tg_it_x;
          std::deque<std::pair<double, double> >::iterator tg_it_y;
          for( tg_it_x = target_x.begin(), tg_it_y = target_y.begin();
               tg_it_x < target_x.end();
               tg_it_x++, tg_it_y++) {
            target_plot_data_x.push_back(*tg_it_x);
            target_plot_data_y.push_back(*tg_it_y);
          }
          std::deque<std::pair<double, double> >::iterator path_it_x;
          std::deque<std::pair<double, double> >::iterator path_it_y;
          for( path_it_x = path_x.begin(), path_it_y = path_y.begin();
               path_it_x < path_x.end();
               path_it_x++, path_it_y++) {
            path_plot_data_x.push_back(*path_it_x);
            path_plot_data_y.push_back(*path_it_y);
          }
          multi_data.push_back(std::make_pair(target_plot_data_x, options_target));
          multi_data.push_back(std::make_pair(target_plot_data_y, options_target));
          multi_data.push_back(std::make_pair(path_plot_data_x, options));
          multi_data.push_back(std::make_pair(path_plot_data_y, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data_x.clear();
          target_plot_data_y.clear();
          path_plot_data_x.clear();
          path_plot_data_y.clear();
          multi_data.clear();
        }
        usleep(0.01*1e6);

        /////////////////////////////////////////////////////////////////////////
        /// 時間-速度x/yプロット
        /////////////////////////////////////////////////////////////////////////
        // 左0詰め4桁設定
        {
          numstr << "'images/trapezoid-5251525/datafile_xy/time_velocity/graph_"
                 << std::setfill('0') << std::setw(4) << index << ".png'";
          std::cerr << "drawing " << numstr.str() << "..." << std::endl;
          gpserver.set("output " +  numstr.str());
          numstr.str("");
          gpserver.set("style data lp");
          gpserver.set("xzeroaxis");
          // gpserver.set("noautoscale");
          gpserver.set("xlabel 'time'");
          gpserver.set("ylabel 'X/Y-velocity'");
          numstr << "0.0 :" << last_time_clock+0.5;
          gpserver.set("xrange ["+ numstr.str() + "]");
          numstr.str("");
          numstr << -tg_x.v_limit() << ":" << tg_x.v_limit();
          gpserver.set("yrange ["+ numstr.str() + "]");
          numstr.str("");
          gpserver.set("nokey");
          gpserver.flush();
          std::deque<std::pair<double, double> >::iterator tg_it_vx;
          std::deque<std::pair<double, double> >::iterator tg_it_vy;
          for( tg_it_vx = target_vx.begin(), tg_it_vy = target_vy.begin();
               tg_it_vx < target_vx.end();
               tg_it_vx++, tg_it_vy++) {
            target_plot_data_vx.push_back(*tg_it_vx);
            target_plot_data_vy.push_back(*tg_it_vy);
          }
          std::deque<std::pair<double, double> >::iterator path_it_vx;
          std::deque<std::pair<double, double> >::iterator path_it_vy;
          for( path_it_vx = path_vx.begin(), path_it_vy = path_vy.begin();
               path_it_vx < path_vx.end();
               path_it_vx++, path_it_vy++ ) {
            path_plot_data_vx.push_back(*path_it_vx);
            path_plot_data_vy.push_back(*path_it_vy);
          }
          multi_data.push_back(std::make_pair(target_plot_data_vx, options_target));
          multi_data.push_back(std::make_pair(target_plot_data_vy, options_target));
          multi_data.push_back(std::make_pair(path_plot_data_vx, options));
          multi_data.push_back(std::make_pair(path_plot_data_vy, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data_vx.clear();
          target_plot_data_vy.clear();
          path_plot_data_vx.clear();
          path_plot_data_vy.clear();
          multi_data.clear();
        }
        usleep(0.01*1e6);
      } // end of プロットデータにキュー

      start_time = t;
      start_x = goal_x;
      start_y = goal_y;

    } // end of if(nusv_x.size() >= 3)
  }// end of 目標点生成のサイクル
  cfstrm.close();
}