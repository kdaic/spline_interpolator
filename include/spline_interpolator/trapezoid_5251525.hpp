#ifndef INCLUDE_TRAJECTORY_GENERATOR_HPP_
#define INCLUDE_TRAJECTORY_GENERATOR_HPP_

#include <cstdlib>

#include <string>
#include <sstream>
#include <complex>
#include <math.h>
#include <deque>
#include <stdexcept>

#include <iostream>
#include <iomanip>

#include "spline_interpolator.hpp"

namespace interp {

/// 台形型5251525次軌道生成クラス
class Trapezoid5251525 {
public:

  /// コンストラクタ
  /// @param[in] a_limit 第一加速(減速)度上限値
  /// @param[in] d_limit 第二加速(減速)度上限値
  /// @param[in] v_limit 最大速度リミット
  /// @param[in] asr 第一丸め率
  /// @param[in] dsr 第二丸め率
  /// @param[in] ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率
  Trapezoid5251525 (const double& a_limit=1200,
                    const double& d_limit=1200,
                    const double& v_limit=170,
                    const double& asr=0.8,
                    const double& dsr=0.8,
                    const double& ratio_acc_dec=0.5);

  /// 軌道生成
  /// @param[in] t0 開始時刻(初期値0)
  /// @param[in] tf 目標到達時刻
  /// @param[in] x0 初期位置
  /// @param[in] xf 終端位置
  /// @param[in] v0 初期速度
  /// @param[in] vf 終端速度
  /// @brief 軌道パラメータを算出する
  /// @detail 以下の計算を順に行う @n
  /// 1. 最大速度方向の初期設定 @n
  /// 2. 最速軌道の最大速度と最短時間の算出 @n
  /// 3. 到達限界・速度反転領域の判定 @n
  /// @return 移動時間(tf-t0) @n
  double generate_path(const double& t0, const double& tf,
                       const double& x0, const double& xf,
                       const double& v0, const double& vf );

  /// デストラクタ
  ~Trapezoid5251525(){};

  /// 軌道出力
  /// @param[in] t 入力時刻
  /// @return 入力時刻のときの位置・速度・加速度
  /// @exception 軌道生成が実施されていない
  int pop(const double& t, double& xt, double& vt, double& at);

  /// 終端時刻
  /// @return 終端時刻
  const double finish_time();

  /// リミット速度
  /// @return リミット速度
  const double v_limit();

private:
  /// 0. 入力範囲のチェック
  void input_check();

  /// 1. 最大速度方向sign_の初期設定
  /// @return sign_の初期設定値
  double calc_initial_v_max_direction_sign();

  /// 2. 最速軌道の最大速度と最短時間の算出
  /// @pararm 最速軌道を構成する第一加速度
  /// @pararm 最速軌道を構成する第二加速度
  void calc_fastest_parameter(const double& a_max, const double& d_max);

  /// 3. 到達限界・速度反転領域の判定
  void judge_reach_limitation();

  /// 加速度を、最短時間に対する目標移動時間の比率により、上限値と下限値の内分をとって算出
  void calc_acceleration_with_ratio();

  /// 正方向のv_max算出
  /// @param[in] signA 第一加速(減速)度方向
  /// @param[in] signD 第二加速(減速)度方向
  /// @@aram[out] dT3 等速区間の時間
  /// @param[out] ret 成功:true/失敗:false
  /// @return v_max 等速度
  double internal_calc_v_max_and_dT3(const double& signA, const double& signD,
                                     double& dT3, bool& ret);

  /// 5. 最大速度v_maxと等速移動時間dT3の算出
  void calc_v_max_and_dT3();

  /// 6. 軌道パラメータ算出
  void set_parameter();


  /// 入力値 ////////////////////////////////////////////////////////////////

  /// 初期位置
  double x0_;

  /// 初期速度
  double v0_;

  /// 終端位置
  double xf_;

  /// 終端速度
  double vf_;

  /// 開始時間
  double t0_;

  /// 目標到達時間
  double tf_;


  /// 既存値 ////////////////////////////////////////////////////////////////

  /// 第一加速(減速)度リミット上限値
  const double a_limit_;

  /// 第二加速(減速)度リミット上限値
  const double d_limit_;

  /// 第一＆第二加速度上限値に対する下限値の比率
  const double ratio_acc_dec_;

  /// 第一加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)
  const double a_lower_limit_;

  /// 第二加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)
  const double d_lower_limit_;

  /// 最大速度リミット
  const double v_limit_;

  /// 第一丸め率
  const double asr_;

  /// 第二丸め率
  const double dsr_;


  /// 制御値 ////////////////////////////////////////////////////////////////

  /// 第一加速(減速)度
  double a_max_;

  /// 第二加速(減速)度
  double d_max_;

  /// 最大速度方向
  double sign_;

  /// 第一加速(減速)度方向
  double signA_;

  /// 第二加速(減速)度方向
  double signD_;

  /// 最大速度
  double v_max_;

  /// 移動距離
  double xd_;

  /// Step1,3移動時間
  double dT1_;

  /// Step2移動時間
  double dT2_;

  /// Step4移動時間
  double dT3_;

  /// Step5,7移動時間
  double dT4_;

  /// Step6移動時間
  double dT5_;

  /// 合計時間
  double dT_total_;

  /// Step1終端時刻＆Step2開始時刻
  double t1_;

  /// Step1終端位置＆Step2開始位置
  double x1_;

  /// Step1終端速度＆Step2開始速度
  double v1_;

  /// Step2終端時刻＆Step3開始時刻
  double t2_;

  /// Step2終端位置＆Step3開始位置
  double x2_;

  /// Step2終端速度＆Step3開始速度
  double v2_;

  /// Step3終端時刻＆Step4開始時刻
  double t3_;

  /// Step3終端位置＆Step4開始位置
  double x3_;

  /// Step3終端速度＆Step4開始速度
  double v3_;

  /// Step4終端時刻＆Step5開始時刻
  double t4_;

  /// Step4終端位置＆Step5開始位置
  double x4_;

  /// Step4終端速度＆Step5開始速度
  double v4_;

  /// Step5終端時刻＆Step6開始時刻
  double t5_;

  /// Step5終端位置＆Step6開始位置
  double x5_;

  /// Step5終端速度＆Step6開始速度
  double v5_;

  /// Step6終端時刻＆Step7開始時刻
  double t6_;

  /// Step6終端位置＆Step7開始位置
  double x6_;

  /// Step6終端速度＆Step7開始速度
  double v6_;

  /// Step7終端時刻
  double t7_;

  /// 最速軌道の最大速度
  double v_max_fastest_;

  /// 最速軌道の最短時間
  double tf_fastest_;

  /// 軌道生成済みフラグ(生成済みならtrue)
  bool is_generated_;

  /// 移動なしフラグ(速度0のまま同じ位置に停止していたらtrue)
  bool no_movement_;
};


///////////////////////////////////////////////////////////////////////////////

class TrapezoidalInterpolator : public SplineInterpolator
{
public:
  /// コンストラクタ
  TrapezoidalInterpolator( TrapezoidConfigQueue& );

  /// コンストラクタ
  /// @param[in] a_limit 第一加速(減速)度上限値の参照
  /// @param[in] d_limit 第二加速(減速)度上限値の参照
  /// @param[in] v_limit 最大速度リミットの参照
  /// @param[in] asr 第一丸め率の参照
  /// @param[in] dsr 第二丸め率の参照
  /// @param[in] ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率の参照
  TrapezoidalInterpolator( const double& a_limit,
                           const double& d_limit,
                           const double& v_limit,
                           const double& asr,
                           const double& dsr,
                           const double& ratio_acc_dec );

  /// デストラクタ
  ~TrapezoidalInterpolator();

  /// 時刻, 位置キューのスプライン軌道の生成
  /// @param[in] target_tp_queue 目標時刻, 位置キュー
  /// @param[in] vs              開始速度 (default: 0.0)
  /// @param[in] vf              終端速度 (default: 0.0)
  /// @return
  /// - SPLINE_SUCCESS
  /// @details
  /// 入力は以下の様な位置・時刻 TimePosition のキュー
  ///
  /// ```
  /// (ts, xs (, vs, as)), (t1, x1), (t2, x2), ..., (tf, xf(, vf, af))
  /// ```
  ///
  /// それと初期＆終端の速度,加速度=0(躍度=0)を入力設定する
  ///
  /// ```
  /// (ts, xs, vs,   as=0, js=0),
  /// (t1, x1, v1=?, a1=?, j1=?),
  /// (t2, x2, v2=?, a2=?, j2=?),
  ///  ...,
  /// (tf, xf, vf,   af=0, jf=0)
  /// ```
  ///
  /// 補間器は中間の (v1,a1,j1), (v2,a2,j2),.. を自動で補間計算する.
  virtual RetCode generate_path( const TPQueue& target_tp_queue,
                                 const double vs=0.0, const double vf=0.0,
                                 const double as=0.0, const double af=0.0);

  /// 時刻, 位置(, 速度)キューからスプライン軌道を生成
  /// @param[in] target_tpva_queue 目標の時刻, 位置(, 速度, 加速度)のキュー
  /// @return
  /// - SPLINE_SUCCESS
  virtual RetCode generate_path( const TPVAQueue& target_tpva_queue );

  /// 開始＆終端の時刻, 位置(, 速度, 加速度)からスプライン軌道を生成
  /// @param[in] xs 開始位置
  /// @param[in] xf 終端位置
  /// @param[in] vs 開始速度 (default: 0.0)
  /// @param[in] vf 終端速度 (default: 0.0)
  /// @param[in] as 開始加速度 (default: 0.0)
  /// @param[in] af 終端加速度 (default: 0.0)
  /// @return
  /// - SPLINE_SUCCESS and total interval time (tf - ts)
  /// 位置 & 速度 & 加速度のみを入力し、時刻は与えない \n
  ///
  /// ```
  /// (xs, vs, as), (xf, vf, af)
  /// ```
  ///
  /// 最小合計移動時間間隔 dT(ts=0, tf) は内部で自動計算される. \n
  /// これは速度・加速度制約下の 100% 最小時間スプライン軌道を意味する
  virtual RetCode generate_path_from_pva( const double& xs,     const double& xf,
                                          const double& vs,     const double& vf,
                                          const double& as=0.0, const double& af=0.0 );

  /// 生成済み軌道から入力時刻の位置, 速度, 加速度を出力
  /// @param[in] t 入力時刻
  /// @return その入力時刻でのTimePVA(時刻,(位置,速度,加速度))の出力
  /// @exception
  /// - NotSplineGenerated : spline-path is not genrated
  /// - TimeOutOfRange : time is not within the range of generated spline-path
  virtual const TimePVA pop( const double& t );

};

}

#endif // _TRAJECTORY_GENERATOR_HPP_
