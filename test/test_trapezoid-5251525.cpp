#include "trapezoid_5251525.hpp"
#include "non_uniform_rounding_spline.hpp"
#include "test/util/gnuplot_realtime.hpp"
#include "test/util/test_graph_plot.hpp"
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
  /// @param dec_limit 最大減速度リミット(デフォルト:1200)
  /// @param vel_limit 最大速度リミット(デフォルト:170)
  /// @param smoothing_rate 丸め率(デフォルト:0.8)
  PlotPtoPGraph(const std::string& output_path="./",
                const double cycle=0.05,
                const double acc_limit=1200,
                const double dec_limit=1200,
                const double vel_limit=170,
                const double smoothing_rate=0.8,
                const std::string& label= "") :
    output_path_(output_path),
    cycle_(cycle),
    acc_limit_(acc_limit), dec_limit_(dec_limit),
    vel_limit_(vel_limit),
    smoothing_rate_(smoothing_rate),
    label_(label) {};

  /// デストラクタ
  ~PlotPtoPGraph(){};

  /// 軌道生成＆グラフプロット
  /// @param start_set 開始点の位置-時間-速度のセット

  /// @param goal_set 終端点の位置-時間-速度のセット
  /// @details 各セットの軌道を生成し、
  /// 時間-位置,時間-速度,位置-速度のグラフを出力 @n
  void plot(const std::vector<TimePVA>& start_set,
            const std::vector<TimePVA>& goal_set) {
    // 描画入力目標点のバッファ
    TPVAQueue target_tpva;

    // 描画軌道補間点のバッファ
    TPVAQueue interp_path_tpva;

    // gnuplot描画クラス
    TestGraphPlot test_gp;

    std::vector<std::string> gpserver_set_list_tp;
    std::vector<std::string> gpserver_set_list_tv;
    std::vector<std::string> gpserver_set_list_pv;

    // 軌道１セット分の開始点
    TimePVA start;
    // 軌道１セット分の目標到達点
    TimePVA goal;
    /// 各軌道セットについて軌道生成＆出力
    for(unsigned int index=0; index < start_set.size(); index++) {
      start = start_set[index];
      goal  = goal_set[index];
      // 軌道生成
      Trapezoid5251525 tg(acc_limit_, dec_limit_,
                          vel_limit_,
                          smoothing_rate_, smoothing_rate_);
      double dT_total = tg.generate_path(start.time,  goal.time,
                                         start.P.pos, goal.P.pos,
                                         start.P.vel, goal.P.vel );
      std::cout << "dT_total:" << dT_total;

      // 開始目標点
      target_tpva.push( TimePVA(
                          start.time,
                          PosVelAcc(start.P.pos, start.P.vel, start.P.acc) ) );

      // 終端目標点
      target_tpva.push( TimePVA(
                          start.time + dT_total,
                          PosVelAcc(goal.P.pos, goal.P.vel, goal.P.acc) ) );

      double xt, vt, at;

      // 生成した軌道(目標点2点間を結ぶ軌道)をプロットデータにキュー
      for(double t=start.time; t < tg.finish_time(); t+=cycle_) {

        tg.pop(t, xt, vt, at);
        // 描画軌道補間点のバッファ追加
        interp_path_tpva.push( TimePVA( t, PosVelAcc(xt, vt, at) ) );
      }

      // 最後に目標到達点データを加える
      tg.pop(tg.finish_time(), xt, vt, at);
      interp_path_tpva.push( TimePVA( tg.finish_time(), PosVelAcc(xt, vt, at) ) );

      // ディレクトリ生成
      const std::string mkdir_str = "mkdir -p " + output_path_;
      FILE* mkdir = popen( mkdir_str.c_str(), "re" );
      pclose(mkdir);
      // データログ出力
      test_gp.dump_csv( interp_path_tpva,
                        output_path_,
                        index,
                        label_ );

      // グラフプロット
      test_gp.plot_tp_tv_pv(target_tpva,
                            interp_path_tpva,
                            output_path_,
                            output_path_,
                            output_path_,
                            index,
                            gpserver_set_list_tp,
                            gpserver_set_list_tv,
                            gpserver_set_list_pv,
                            label_
                            );

      // PtoP ２点間１セットごとにクリア
      target_tpva.clear();
      interp_path_tpva.clear();
    }
    usleep(0.01*1e6);
  };

  /// グラフ＆データファイル出力先パス
  const std::string output_path_;
  /// プロット周期
  const double cycle_;
  /// 最大加速リミット
  const double acc_limit_;
  /// 最大減速リミット
  const double dec_limit_;
  /// 最大速度リミット
  const double vel_limit_;
  /// 丸め率
  const double smoothing_rate_;
  /// ラベル
  const std::string label_;
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
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable", 0.001);

  // 既に存在するデータ&画像を削除
  FILE* rm_time_position_velocity_log = popen("rm -f images/trapezoid-5251525/reachable/*.csv", "re");
  pclose(rm_time_position_velocity_log);
  FILE* rm_images = popen("rm -f images/trapezoid-5251525/reachable/*.png", "re");
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
  FILE* rm_images = popen("rm -f images/trapezoid-5251525/reachable2/*.png", "re");
  pclose(rm_images);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable2",
                                0.01,               // cycle
                                31.415926535897931, // acc_limit
                                31.415926535897931, // dec_limit
                                3.1415926535897931, // v_limit
                                0.8);               // smoothing_rate
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
  FILE* rm_time_position_images = popen("rm -f images/trapezoid-5251525/reachable3/*.png", "re");
  pclose(rm_time_position_images);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable3",
                                0.01,        // cycle
                                13.17723585, // acc_limit
                                13.17723585, // dec_limit
                                6.44026494,  // v_limit
                                0.01);       // smoothing_rate
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
  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm -f images/trapezoid-5251525/reachable_L1/*.png", "re");
  pclose(rm_time_position_images);
  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable_L1",
                                0.01, // cycle
                                1.2,  // acc_limit
                                1.2,  // dec_limit
                                1.0,  // v_limit
                                0.8); // smoothing_rate

  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);
}




TEST(TrackingTest, reachable4) {
  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;

  // 0.
  start.time  = 0.0;
  start.P.pos = 0.37;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.0;
  // goal.time  = 0.290472121528706;
  goal.P.pos = 0.375;
  goal.P.vel = -0.005862398499446;
  goal_set.push_back(goal);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph("images/trapezoid-5251525/reachable4",
                                0.001,               // cycle
                                0.923349448054940,  // acc_limit
                                0.921556254253312,  // dec_limit
                                0.131365013138030,  // v_limit
                                0.00,               // smoothing_rate
                                "test01-");         // label
  // ディレクトリ作成
  FILE* mkdir = popen("mkdir -p images/trapezoid-5251525/reachable4", "re");
  pclose(mkdir);

  // 既に存在する画像を削除
  FILE* rm_time_position_images = popen("rm -f images/trapezoid-5251525/reachable4/*.png", "re");
  pclose(rm_time_position_images);

  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);

  // -----------------------------------------------
  start_set.clear();
  goal_set.clear();


  // // 3. 01
  start.time = 0.0;
  start.P.pos = 1.000000000000000;
  start.P.vel = 0.000000000000000;
  start_set.push_back(start);
  goal.time  = 0.290472121528706;
  // goal.time  = 0.290472121528706;
  goal.P.pos = 0.970691715357122;
  goal.P.vel = -0.089127359654605;
  goal_set.push_back(goal);
  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph3_01("images/trapezoid-5251525/reachable4",
                                 0.001,               // cycle
                                 5.412357689687719,  // acc_limit
                                 5.401846602816129,  // dec_limit
                                 2.400179894310761,  // v_limit
                                 0.00,               // smoothing_rate
                                 "test02-01-");      // label

  // 軌道生成＆グラフ出力
  plot_ptop_graph3_01.plot(start_set, goal_set);

  //
  start_set.clear();
  goal_set.clear();


  // // 3. 01
  start.time = 0.290472121528706;
  start.P.pos = 0.970691715357122;
  start.P.vel = -0.089127359654605;
  start_set.push_back(start);
  goal.time  = 0.926451701915314;
  // goal.time  = 0.290472121528706;
  goal.P.pos = 0.974642595936322;
  goal.P.vel = 0.000000000000000;
  goal_set.push_back(goal);
  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph3_02("images/trapezoid-5251525/reachable4",
                                 0.005,              // cycle
                                 0.454701321864866,  // acc_limit
                                 0.296208475089815,  // dec_limit
                                 2.254393935200920,  // v_limit
                                 0.00,               // smoothing_rate
                                 "test02-02-");      // label

  // 軌道生成＆グラフ出力
  plot_ptop_graph3_02.plot(start_set, goal_set);

  // -----------------------------------------------
  start_set.clear();
  goal_set.clear();

  // // 6.
  start.time = 0.0;
  start.P.pos = 0.0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.0;
  // goal.time  = 0.290472121528706;
  goal.P.pos = 0.182710741389627;
  goal.P.vel = 0.024900462983757;
  goal_set.push_back(goal);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph6("images/trapezoid-5251525/reachable4",
                                 0.001,              // cycle
                                 33.741172443164139, // acc_limit
                                 33.675645289374003, // dec_limit
                                 2.400179894310761,  // v_limit
                                 0.00,               // smoothing_rate
                                 "test03-");         // label

  // 軌道生成＆グラフ出力
  plot_ptop_graph6.plot(start_set, goal_set);

}

/// @test
TEST(TrackingTest, reachable5)
{
  std::string outdir = "images/trapezoid-5251525/reachable5";
   // ディレクトリ作成
  std::string mkdir_cmd = "mkdir -p " + outdir;
  FILE* mkdir = popen( mkdir_cmd.c_str(), "re" );
  pclose(mkdir);
  // 既に存在する画像を削除
  std::string rmdir_cmd = "rm_-f " + outdir + "/*.png";
  FILE* rm_time_position_images = popen(rmdir_cmd.c_str(), "re");
  pclose(rm_time_position_images);

  // 開始-目標到達の軌道セット
  std::vector<TimePVA> start_set;
  std::vector<TimePVA> goal_set;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;

  // 1.
  start.time  = 0.0;
  start.P.pos = 1.0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.0;
  goal.P.pos = 0.996194698091746;
  goal.P.vel = -0.081634405782459;
  goal_set.push_back(goal);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph( outdir,
                                 0.0001,              // cycle
                                 44.710896029548501,  // acc_limit
                                 44.710896029548621,  // dec_limit
                                 1.507059421063559,  // v_limit
                                 0.00,               // smoothing_rate
                                 "test01-");         // label

  // 軌道生成＆グラフ出力
  plot_ptop_graph.plot(start_set, goal_set);

  // -----------------------------------------------
  start_set.clear();
  goal_set.clear();

  // 2.
  start.time  = 0.0;
  start.P.pos = 1.0;
  start.P.vel = 0.0;
  start_set.push_back(start);
  goal.time  = 0.092896849441572;
  goal.P.pos = 0.996194698091746;
  goal.P.vel = -0.081634405782459;
  goal_set.push_back(goal);

  // 最大加速度リミット、最大速度リミット、丸め率、グラフ＆データ出力先パス設定
  PlotPtoPGraph plot_ptop_graph2( outdir,
                                  0.0001,              // cycle
                                  44.710896029548501,  // acc_limit
                                  44.710896029548621,  // dec_limit
                                  1.507059421063559,  // v_limit
                                  0.00,               // smoothing_rate
                                  "test02-");         // label

  // 軌道生成＆グラフ出力
  plot_ptop_graph2.plot(start_set, goal_set);

  // -----------------------------------------------
  start_set.clear();
  goal_set.clear();
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
  const unsigned int target_tp_num = 5;
  // 描画軌道補間点の数
  const unsigned int path_num = static_cast<unsigned int>((target_tp_num)*PATH_CYCLE/CYCLE);

  // 描画入力目標点のバッファ
  TPVAQueue target_tpva;

  // 描画軌道補間点のバッファ
  TPVAQueue interp_path_tpva;

  // プロットコマンド入力文字列
  std::stringstream numstr;
  std::vector<std::string> gpserver_set_list_tp;
  std::vector<std::string> gpserver_set_list_tv;
  std::vector<std::string> gpserver_set_list_pv;

  // グラフ画像出力ディレクトリ名
  const std::string output_dir_tp = "images/trapezoid-5251525/random_plot/time_position";
  const std::string output_dir_tv = "images/trapezoid-5251525/random_plot/time_velocity";
  const std::string output_dir_pv = "images/trapezoid-5251525/random_plot/postion_velocity";

  // ディレクトリ作成
  const std::string mkdir_tp_str = "mkdir -p " + output_dir_tp;
  const std::string mkdir_tv_str = "mkdir -p " + output_dir_tv;
  const std::string mkdir_pv_str = "mkdir -p " + output_dir_pv;
  FILE* mkdir_tp = popen(mkdir_tp_str.c_str(), "re");
  FILE* mkdir_tv = popen(mkdir_tv_str.c_str(), "re");
  FILE* mkdir_pv = popen(mkdir_pv_str.c_str(), "re");
  pclose(mkdir_tp);
  pclose(mkdir_tv);
  pclose(mkdir_pv);

  // 既に存在する画像を削除
  const std::string rm_tp_images_str = "rm -f " + output_dir_tp + "/*.png";
  const std::string rm_tv_images_str = "rm -f " + output_dir_tv + "/*.png";
  const std::string rm_pv_images_str = "rm -f " + output_dir_pv + "/*.png";
  FILE* rm_tp_images = popen(rm_tp_images_str.c_str(), "re");
  FILE* rm_tv_images = popen(rm_tv_images_str.c_str(), "re");
  FILE* rm_pv_images = popen(rm_pv_images_str.c_str(), "re");
  pclose(rm_tp_images);
  pclose(rm_tv_images);
  pclose(rm_pv_images);

  // 目標位置(初期化=0.0)
  double target_position=0.0;
  // 時間・位置・速度のバッファ
  NonUniformRoundingSpline nusv;
  // gnuplot描画クラス
  TestGraphPlot test_gp;
  // 軌道１セット分の開始点
  TimePVA start;
  // 軌道１セット分の目標到達点
  TimePVA goal;
  // 目標点プロットと軌道生成プロットのタイミング調整用
  TimePVA target_pop;

  // 目標点生成のサイクル
  int index = 0;
  for (int tpindex=0; tpindex < POINT_NUM; tpindex++) {

    if(tpindex == 0) {
      target_position = INIT_POSITION;
    } else {
      // 目標位置を乱数で生成
      target_position = rand_regularized(MU, SIGMA);
    }

    // 目標位置から時間・位置・速度の点を生成(3点分貯める)
    nusv.push( tpindex*PATH_CYCLE, target_position );

    // 開始３点未満は 不均一丸みスプラインの速度計算未実施のため
    // まだ軌道生成できる(目標速度をもった)目標goal点はpopできないためcontinue
    if( nusv.size() < 3 ) {
      continue;
    }

    start = goal;
    goal = target_pop;
    //
    target_pop = nusv.pop();
    //
    target_tpva.push( TimePVA( (tpindex-2) * PATH_CYCLE,
                               PosVelAcc(target_pop.P.pos, target_pop.P.vel) ) );

    // 開始目標点4点未満は 不均一丸みスプラインの速度計算後の開始-終了2点がpopされていない、
    // まだ軌道生成できるstart-goalのセットはないためcontinue。
    // 4点目では目標点が2点popされた状態だが、
    // 実際の軌道生成のタイミングは5点目(目標点が3点ある状態)
    if( tpindex < 4) {
      continue;
    }

    // 軌道生成
    Trapezoid5251525 tg;
    tg.generate_path(start.time,  goal.time,
                     start.P.pos, goal.P.pos,
                     start.P.vel, goal.P.vel);
    double xt, vt, at;

    // リアルタイム軌道再生の様子を1枚1枚グラフで作成
    // 目標点2点間軌道をサイクル時間で刻んで軌道上点群をプロットデータinterp_path_tpvaとしてキュー
    for(double t=start.time; t <= tg.finish_time()+1e-12; t+=CYCLE, index++) {

      tg.pop(t, xt, vt, at);
      if(interp_path_tpva.size() > path_num) {
        interp_path_tpva.pop_delete();
      }
      // 描画軌道補間点のバッファ追加
      interp_path_tpva.push( TimePVA( t, PosVelAcc(xt, vt, at) ) );

      // 描画入力目標点のバッファ除去
      if(target_tpva.front().time < interp_path_tpva.front().time ) {
        target_tpva.pop_delete();
      }

      // 時間-位置グラフをgnuplotでプロットする前にセットするオプションを設定
      numstr << interp_path_tpva.front().time << ":"
             << interp_path_tpva.front().time + (target_tp_num+4)*PATH_CYCLE;
      gpserver_set_list_tp.push_back("xrange ["+ numstr.str() + "]");
      gpserver_set_list_tp.push_back("yrange [-30:30]");

      // 時間-速度グラフをgnuplotでプロットする前にセットするオプションを設定
      gpserver_set_list_tv.push_back("xrange ["+ numstr.str() + "]");
      numstr.str("");
      numstr << -tg.v_limit()*0.7 << ":" << tg.v_limit()*0.7 ;
      gpserver_set_list_tv.push_back("yrange ["+ numstr.str() + "]");

      // 位置-速度グラフをgnuplotでプロットする前にセットするオプションを設定
      gpserver_set_list_pv.push_back("xrange [-30:30]");
      gpserver_set_list_pv.push_back("yrange ["+ numstr.str() + "]");
      numstr.str("");

      // グラフプロット
      test_gp.plot_tp_tv_pv(target_tpva,
                            interp_path_tpva,
                            output_dir_tp,
                            output_dir_tv,
                            output_dir_pv,
                            index,
                            gpserver_set_list_tp,
                            gpserver_set_list_tv,
                            gpserver_set_list_pv,
                            "",
                            true
                           );

      gpserver_set_list_tp.clear();
      gpserver_set_list_tv.clear();
      gpserver_set_list_pv.clear();

      usleep(0.01*1e6);
    } // end of プロットデータにキュー

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
  const unsigned int Id_X = 0;
  const unsigned int Id_Y = 1;

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
  const unsigned int path_num      = static_cast<unsigned int>((target_num)     *PATH_CYCLE/CYCLE);
  // const unsigned int path_time_num = static_cast<unsigned int>((target_time_num)*PATH_CYCLE/CYCLE);
  // 描画入力目標点のバッファ
  std::vector<TPVAQueue> target_tpva_list;
  target_tpva_list.resize(2);
  std::deque<double> target_pos_x;
  std::deque<double> target_pos_y;

  // プロットコマンド入力文字列
  std::stringstream numstr;
  std::vector<std::string> gpserver_set_list_tp;
  std::vector<std::string> gpserver_set_list_tv;
  std::vector<std::string> gpserver_set_list_pv;
  std::vector<std::string> gpserver_set_list_xy;

  // 描画軌道補間点のバッファ
  std::vector<TPVAQueue> interp_path_tpva_list;
  interp_path_tpva_list.resize(2);
  std::deque<double> interp_path_pos_x;
  std::deque<double> interp_path_pos_y;

  // gnuplot描画クラス
  TestGraphPlot test_gp;

  // 時間・位置・速度のバッファ
  NonUniformRoundingSpline nusv_x(INIT_POSITION);
  NonUniformRoundingSpline nusv_y(INIT_POSITION);
  // 軌道１セット分の開始点
  TimePVA start_x;
  TimePVA start_y;
  // 軌道１セット分の目標到達点
  TimePVA goal_x;
  TimePVA goal_y;

  TimePVA target_pop_x;
  TimePVA target_pop_y;

  // グラフ画像出力ディレクトリ名
  const std::string output_dir_tp = "images/trapezoid-5251525/xy_plot/time_position";
  const std::string output_dir_tv = "images/trapezoid-5251525/xy_plot/time_velocity";
  const std::string output_dir_pv = "images/trapezoid-5251525/xy_plot/postion_velocity";
  const std::string output_dir_xy = "images/trapezoid-5251525/xy_plot/xy";

  // ディレクトリ作成
  const std::string mkdir_tp_str = "mkdir -p " + output_dir_tp;
  const std::string mkdir_tv_str = "mkdir -p " + output_dir_tv;
  const std::string mkdir_pv_str = "mkdir -p " + output_dir_pv;
  const std::string mkdir_xy_str = "mkdir -p " + output_dir_xy;
  FILE* mkdir_tp = popen(mkdir_tp_str.c_str(), "re");
  FILE* mkdir_tv = popen(mkdir_tv_str.c_str(), "re");
  FILE* mkdir_pv = popen(mkdir_pv_str.c_str(), "re");
  FILE* mkdir_xy = popen(mkdir_xy_str.c_str(), "re");
  pclose(mkdir_tp);
  pclose(mkdir_tv);
  pclose(mkdir_pv);
  pclose(mkdir_xy);

  // ファイル出力
  const std::string prefix_pva_label =  "XY-";
  const std::string dump_csv_filepath = output_dir_xy + "/time_" + prefix_pva_label + "position.csv";
  std::fstream cfstrm(dump_csv_filepath.c_str(), std::ios::out);
  if (cfstrm.fail()) {
    std::cerr << "cannot open output temporary file."<< std::endl;
    FAIL();
  }

  // 桁固定
  cfstrm << std::fixed << std::setprecision(8);
  // 既に存在する画像を削除
  const std::string rm_tp_images_str = "rm -f " + output_dir_tp + "/*.png";
  const std::string rm_tv_images_str = "rm -f " + output_dir_tv + "/*.png";
  const std::string rm_pv_images_str = "rm -f " + output_dir_pv + "/*.png";
  const std::string rm_xy_images_str = "rm -f " + output_dir_xy + "/*.png";
  FILE* rm_tp_images = popen(rm_tp_images_str.c_str(), "re");
  FILE* rm_tv_images = popen(rm_tv_images_str.c_str(), "re");
  FILE* rm_pv_images = popen(rm_pv_images_str.c_str(), "re");
  FILE* rm_xy_images = popen(rm_xy_images_str.c_str(), "re");
  pclose(rm_tp_images);
  pclose(rm_tv_images);
  pclose(rm_pv_images);
  pclose(rm_xy_images);

  // 目標点生成のサイクル
  int index = 0;
  for (unsigned int tpindex=0; tpindex < target_num; tpindex++) {

    // 目標位置から時間・位置・速度の点を生成(3点分貯める)
    nusv_x.push( tpindex*PATH_CYCLE, target_position_x[tpindex] );
    nusv_y.push( tpindex*PATH_CYCLE, target_position_y[tpindex] );

    // 開始３点未満は 不均一丸みスプラインの速度計算未実施のため
    // まだ軌道生成できる(目標速度をもった)目標goal点はpopできないためcontinue
    if( nusv_x.size() < 3 ) {
      continue;
    }

    start_x = goal_x;
    start_y = goal_y;
    goal_x  = target_pop_x;
    goal_y  = target_pop_y;
    //
    target_pop_x = nusv_x.pop();
    target_pop_y = nusv_y.pop();
    //
    target_tpva_list[Id_X].push( TimePVA( (tpindex-2) * PATH_CYCLE,
                                   PosVelAcc(target_pop_x.P.pos, target_pop_x.P.vel)
                                 )
                               );
    target_tpva_list[Id_Y].push( TimePVA( (tpindex-2) * PATH_CYCLE,
                                   PosVelAcc(target_pop_y.P.pos, target_pop_y.P.vel)
                                 )
                               );

    target_pos_x.push_back( target_pop_x.P.pos );
    target_pos_y.push_back( target_pop_y.P.pos );

    // 開始目標点4点未満は 不均一丸みスプラインの速度計算後の開始-終了2点がpopされていない、
    // まだ軌道生成できるstart-goalのセットはないためcontinue。
    // 4点目では目標点が2点popされた状態だが、
    // 実際の軌道生成のタイミングは5点目(目標点が3点ある状態)
    if( tpindex < 4) {
      continue;
    }

    // 軌道生成
    Trapezoid5251525 tg_x;
    tg_x.generate_path(start_x.time,  goal_x.time,
                       start_x.P.pos, goal_x.P.pos,
                       start_x.P.vel, goal_x.P.vel);

    Trapezoid5251525 tg_y;
    tg_y.generate_path(start_y.time,  goal_y.time,
                       start_y.P.pos, goal_y.P.pos,
                       start_y.P.vel, goal_y.P.vel);

    double pt_x, vt_x, at_x;
    double pt_y, vt_y, at_y;

    // リアルタイム軌道再生の様子を1枚1枚グラフで作成
    // 目標点2点間軌道をサイクル時間で刻んで軌道上点群をプロットデータinterp_path_tpvaとしてキュー
    for(double t=start_x.time; t <= tg_x.finish_time()+1e-12; t+=CYCLE, index++) {

      tg_x.pop(t, pt_x, vt_x, at_x);
      tg_y.pop(t, pt_y, vt_y, at_y);
      if(interp_path_tpva_list[Id_X].size() > path_num) {
        interp_path_tpva_list[Id_X].pop_delete();
        interp_path_tpva_list[Id_Y].pop_delete();
        interp_path_pos_x.pop_front();
        interp_path_pos_y.pop_front();
      }
      // 描画軌道補間点のバッファ追加
      interp_path_tpva_list[Id_X].push( TimePVA( t, PosVelAcc(pt_x, vt_x, at_x) ) );
      interp_path_tpva_list[Id_Y].push( TimePVA( t, PosVelAcc(pt_y, vt_y, at_y) ) );
      interp_path_pos_x.push_back( pt_x );
      interp_path_pos_y.push_back( pt_y );

      // 描画入力目標点のバッファ除去
      if(target_tpva_list[Id_X].front().time < interp_path_tpva_list[Id_X].front().time ) {
        target_tpva_list[Id_X].pop_delete();
        target_tpva_list[Id_Y].pop_delete();
        target_pos_x.pop_front();
        target_pos_y.pop_front();
      }

      std::cout << "tpindex : " << tpindex << "/" << target_num-4;

      /////////////////////////////////////////////////////////////////////////
      /// xy平面プロット
      /////////////////////////////////////////////////////////////////////////
      {
        numstr << "xrange ["
               << target_position_x[0]-0.1*range_x << ":" << 1.1*range_x
               << "]";
        gpserver_set_list_xy.push_back(numstr.str());

        numstr.str("");
        numstr << "yrange ["
               << target_position_y[0]-0.1*range_y << ":" << 1.1*range_y
               << "]";
        gpserver_set_list_xy.push_back(numstr.str());
        numstr.str("");

        test_gp.plot(target_pos_x,
                     target_pos_y,
                     interp_path_pos_x,
                     interp_path_pos_y,
                     output_dir_xy,
                     "xy_",
                     index,
                     gpserver_set_list_xy,
                     "x-position",
                     "y-position"
                    );

        gpserver_set_list_xy.clear();
      }

      /////////////////////////////////////////////////////////////////////////
      /// 時間-位置x/y平面データファイル出力
      /////////////////////////////////////////////////////////////////////////
      {
        cfstrm << t << "," << pt_x << "," << pt_y << std::endl;
      }

      /////////////////////////////////////////////////////////////////////////
      /// 時間-位置, 時間-速度, 位置-速度 x/y平面プロット
      /////////////////////////////////////////////////////////////////////////
      // 左0詰め4桁設定
      {
        numstr
          << interp_path_tpva_list[Id_X].front().time
          << ":"
          << (interp_path_tpva_list[Id_X].front().time + (target_time_num+4)*PATH_CYCLE);
        gpserver_set_list_tp.push_back( "xrange [" + numstr.str() + "]" );
        numstr.str("");

        numstr
          << target_position_x[0]-0.1*range_x
          << ":"
          << 1.1*range_x;
        gpserver_set_list_tp.push_back( "yrange [" + numstr.str() + "]" );
        gpserver_set_list_pv.push_back( "xrange [" + numstr.str() + "]" );
        numstr.str("");

        numstr
          << interp_path_tpva_list[Id_X].front().time
          << ":"
          << (interp_path_tpva_list[Id_X].front().time + (target_time_num+4)*PATH_CYCLE);
        gpserver_set_list_tv.push_back( "xrange [" + numstr.str() + "]" );
        numstr.str("");

        numstr
          << -tg_x.v_limit()
          << ":"
          << tg_x.v_limit();
        gpserver_set_list_tv.push_back( "yrange [" + numstr.str() + "]" );
        gpserver_set_list_pv.push_back( "yrange [" + numstr.str() + "]" );
        numstr.str("");

        test_gp.plot_tp_tv_pv( target_tpva_list,
                               interp_path_tpva_list,
                               output_dir_tp,
                               output_dir_tv,
                               output_dir_pv,
                               index,
                               gpserver_set_list_tp,
                               gpserver_set_list_tv,
                               gpserver_set_list_pv,
                               prefix_pva_label
                              );
        gpserver_set_list_tp.clear();
        gpserver_set_list_tv.clear();
        gpserver_set_list_pv.clear();
      }
      usleep(0.01*1e6);

    } // end of プロットデータにキュー

    start_x = goal_x;
    start_y = goal_y;

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
  std::deque<std::pair<double, double> > target_xy;
  std::deque<std::pair<double, double> > target_tx;
  std::deque<std::pair<double, double> > target_tvx;
  std::deque<std::pair<double, double> > target_ty;
  std::deque<std::pair<double, double> > target_tvy;
  // 描画軌道補間点のバッファ
  std::deque<std::pair<double, double> > path_xy;
  std::deque<std::pair<double, double> > path_tx;
  std::deque<std::pair<double, double> > path_tvx;
  std::deque<std::pair<double, double> > path_ty;
  std::deque<std::pair<double, double> > path_tvy;
  // 入力目標点の描画オプション
  plot_options_t options_target_tp;
  options_target_tp["with"] = "point";
  options_target_tp["ps"] = "5.0";
  plot_options_t options;
  options["with"] = "lp";
  // 目標値描画データ
  std::vector<std::pair<double, double> >target_plot_data;
  std::vector<std::pair<double, double> >target_plot_data_x;
  std::vector<std::pair<double, double> >target_plot_data_tvx;
  std::vector<std::pair<double, double> >target_plot_data_y;
  std::vector<std::pair<double, double> >target_plot_data_tvy;
  // 補間点描画データ
  std::vector<std::pair<double, double> >path_plot_data;
  std::vector<std::pair<double, double> >path_plot_data_x;
  std::vector<std::pair<double, double> >path_plot_data_tvx;
  std::vector<std::pair<double, double> >path_plot_data_y;
  std::vector<std::pair<double, double> >path_plot_data_tvy;
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
  FILE* rm_time_position_images = popen("rm -f images/trapezoid-5251525/datafile_xy/time_position/*.png", "re");
  pclose(rm_time_position_images);
  FILE* rm_time_velocity_images = popen("rm -f images/trapezoid-5251525/datafile_xy/time_velocity/*.png", "re");
  pclose(rm_time_velocity_images);
  FILE* rm_xy_images = popen("rm -f images/trapezoid-5251525/datafile_xy/xy/*.png", "re");
  pclose(rm_xy_images);


  // 目標点生成のサイクル
  int index = 0;
  double timecount = target_time[0];
  for (unsigned int tpindex=1; tpindex <= target_num; tpindex++) {
    timecount += target_time[tpindex];
    std::cout << "tpindex : " << tpindex << "/" << target_num;
    if(tpindex > 0) {
      // 目標位置から目標時間・位置・速度の点を生成(3点分貯める)
      nusv_x.push_dT(target_time[tpindex], target_position_x[tpindex]);
      nusv_y.push_dT(target_time[tpindex], target_position_y[tpindex]);
    }

    target_xy.push_back( std::make_pair(target_position_x[tpindex], target_position_y[tpindex]) );
    target_tx.push_back( std::make_pair(timecount,
                                       target_position_x[tpindex]) );
    target_ty.push_back( std::make_pair(timecount,
                                       target_position_y[tpindex]) );

    if(tpindex == 2) {
      start_x = nusv_x.pop();
      start_y = nusv_y.pop();
      target_tvx.push_back( std::make_pair(start_x.time,
                                          start_x.P.vel) );
      target_tvy.push_back( std::make_pair(start_y.time,
                                          start_y.P.vel) );
      continue;
    }

    if(nusv_x.size() > 2) {
      goal_x = nusv_x.pop();
      goal_y = nusv_y.pop();
      std::cerr << "goal_time :" << goal_x.time << std::endl;
      std::cerr << "interval_time : " << target_time[tpindex-2] << std::endl;
      std::cerr << "goal_x :" << goal_x.P.pos << std::endl;
      std::cerr << "goal_y :" << goal_y.P.pos << std::endl;
      target_tvx.push_back( std::make_pair(goal_x.time, goal_x.P.vel) );
      target_tvy.push_back( std::make_pair(goal_x.time, goal_y.P.vel) );

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
        path_xy.push_back(std::make_pair(xt, yt));
        path_tx.push_back(std::make_pair(t, xt));
        path_ty.push_back(std::make_pair(t, yt));
        path_tvx.push_back(std::make_pair(t, v_xt));
        path_tvy.push_back(std::make_pair(t, v_yt));

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

          for(std::deque<std::pair<double, double> >::iterator it = target_xy.begin();
              it < target_xy.end();
              it++) {
            target_plot_data.push_back(*it);
          }
          for(std::deque<std::pair<double, double> >::iterator it = path_xy.begin();
              it < path_xy.end();
              it++) {
            path_plot_data.push_back(*it);
          }
          multi_data.push_back(std::make_pair(target_plot_data, options_target_tp));
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

          std::deque<std::pair<double, double> >::iterator tg_it_tx;
          std::deque<std::pair<double, double> >::iterator tg_it_ty;
          for( tg_it_tx = target_tx.begin(), tg_it_ty = target_ty.begin();
               tg_it_tx < target_tx.end();
               tg_it_tx++, tg_it_ty++) {
            target_plot_data_x.push_back(*tg_it_tx);
            target_plot_data_y.push_back(*tg_it_ty);
          }
          std::deque<std::pair<double, double> >::iterator path_it_tx;
          std::deque<std::pair<double, double> >::iterator path_it_ty;
          for( path_it_tx = path_tx.begin(), path_it_ty = path_ty.begin();
               path_it_tx < path_tx.end();
               path_it_tx++, path_it_ty++) {
            path_plot_data_x.push_back(*path_it_tx);
            path_plot_data_y.push_back(*path_it_ty);
          }
          multi_data.push_back(std::make_pair(target_plot_data_x, options_target_tp));
          multi_data.push_back(std::make_pair(target_plot_data_y, options_target_tp));
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
          std::deque<std::pair<double, double> >::iterator tg_it_tvx;
          std::deque<std::pair<double, double> >::iterator tg_it_tvy;
          for( tg_it_tvx = target_tvx.begin(), tg_it_tvy = target_tvy.begin();
               tg_it_tvx < target_tvx.end();
               tg_it_tvx++, tg_it_tvy++) {
            target_plot_data_tvx.push_back(*tg_it_tvx);
            target_plot_data_tvy.push_back(*tg_it_tvy);
          }
          std::deque<std::pair<double, double> >::iterator path_it_tvx;
          std::deque<std::pair<double, double> >::iterator path_it_tvy;
          for( path_it_tvx = path_tvx.begin(), path_it_tvy = path_tvy.begin();
               path_it_tvx < path_tvx.end();
               path_it_tvx++, path_it_tvy++ ) {
            path_plot_data_tvx.push_back(*path_it_tvx);
            path_plot_data_tvy.push_back(*path_it_tvy);
          }
          multi_data.push_back(std::make_pair(target_plot_data_tvx, options_target_tp));
          multi_data.push_back(std::make_pair(target_plot_data_tvy, options_target_tp));
          multi_data.push_back(std::make_pair(path_plot_data_tvx, options));
          multi_data.push_back(std::make_pair(path_plot_data_tvy, options));

          gpserver.plot(multi_data);
          gpserver.flush();

          target_plot_data_tvx.clear();
          target_plot_data_tvy.clear();
          path_plot_data_tvx.clear();
          path_plot_data_tvy.clear();
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
