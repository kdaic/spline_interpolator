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

namespace interp {

/// 台形型5251525次軌道生成クラス
class Trapezoid5251525 {
public:
  /// コンストラクタ
  Trapezoid5251525 ();

  /// コンストラクタ
  /// @param[in] a_limit 第一加速(減速)度上限値
  /// @param[in] d_limit 第二加速(減速)度上限値
  /// @param[in] v_limit 最大速度リミット
  /// @param[in] asr 第一丸め率 (default : 0.0)
  /// @param[in] dsr 第二丸め率 (default : 0.0)
  /// @param[in] ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率 (default : 1.0)
  Trapezoid5251525 (const double& a_limit,
                    const double& d_limit,
                    const double& v_limit,
                    const double& asr=0.0,
                    const double& dsr=0.0,
                    const double& ratio_acc_dec=1.0);

  /// コピーコンストラクタ
  /// @param[in] src コピー元Trapezoid5251525クラスオブジェクト
  Trapezoid5251525(const Trapezoid5251525& src);

  /// 代入演算子
  /// @param[in] src コピー元Trapezoid5251525クラスオブジェクト
  /// @return *this
  Trapezoid5251525 operator=(const Trapezoid5251525& src);

  /// 構成パラメータの初期化
  /// @param[in] a_limit 第一加速(減速)度上限値
  /// @param[in] d_limit 第二加速(減速)度上限値
  /// @param[in] v_limit 最大速度リミット
  /// @param[in] asr 第一丸め率
  /// @param[in] dsr 第二丸め率
  /// @param[in] ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率
  void initialize (const double& a_limit=1200,
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
  const int pop(const double& t, double& xt, double& vt, double& at) const;

  /// 終端時刻
  /// @return 終端時刻
  const double finish_time();

  /// 入力制限値 ////////////////////////////////////////////////////////////

  /// 移動時間(tf-t0)の最大閾値[sec]
  static const double DT_MAX_LIMIT_;

private:
  /// 0. 入力範囲のチェック
  void input_check();

  /// 1. 最大速度方向sign_の初期設定
  /// @return sign_の初期設定値
  double calc_initial_v_max_direction_sign();

  /// 2. 最速軌道の最大速度と最短時間の算出
  /// @param a_max          最速軌道を構成する第一加速度
  /// @param d_max          最速軌道を構成する第二加速度
  void calc_fastest_parameter( const double& a_max,
                               const double& d_max );

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
  double a_limit_;

  /// 第二加速(減速)度リミット上限値
  double d_limit_;

  /// 第一＆第二加速度上限値に対する下限値の比率
  double ratio_acc_dec_;

  /// 第一加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)
  double a_lower_limit_;

  /// 第二加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)
  double d_lower_limit_;

  /// 最大速度リミット
  double v_limit_;

  /// 第一丸め率
  double asr_;

  /// 第二丸め率
  double dsr_;


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

  /// 最速軌道のdT3
  double dT3_fastest_;

  /// 最速軌道の最短時間
  double tf_fastest_;

  /// 初期化済み(構成パラメータ設定済み)フラグ(済みならtrue)
  bool is_initialized_;

  /// 軌道生成済みフラグ(生成済みならtrue)
  bool is_generated_;

  /// 移動なしフラグ(速度0のまま同じ位置に停止していたらtrue)
  bool no_movement_;

  /// 最速軌道のフラグ(最速軌道ならtrue)
  bool is_fastest_;

public:
  /// 入力値の取得 ////////////////////////////////////////////////////////////////

  /// 初期位置の取得
  const double x0() const;

  /// 初期速度の取得
  const double v0() const;

  /// 終端位置の取得
  const double xf() const;

  /// 終端速度の取得
  const double vf() const;

  /// 開始時間の取得
  const double t0() const;

  /// 目標到達時間の取得
  const double tf() const;

  /// 既存値の取得 ////////////////////////////////////////////////////////////////

  /// 第一加速(減速)度リミット上限値の取得
  const double a_limit() const;

  /// 第二加速(減速)度リミット上限値の取得
  const double d_limit() const;

  /// 第一＆第二加速度上限値に対する下限値の比率の取得
  const double ratio_acc_dec() const;

  /// 第一加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)の取得
  const double a_lower_limit() const;

  /// 第二加速(減速)度リミット下限値(=上限値*ratio_acc_dec_)の取得
  const double d_lower_limit() const;

  /// 最大速度リミットの取得
  const double v_limit() const;

  /// 第一丸め率の取得
  const double asr() const;

  /// 第二丸め率の取得
  const double dsr() const;


  /// 制御値の取得 ////////////////////////////////////////////////////////////////

  /// 第一加速(減速)度の取得
  const double a_max() const;

  /// 第二加速(減速)度の取得
  const double d_max() const;

  /// 最大速度方向の取得
  const double sign() const;

  /// 第一加速(減速)度方向の取得
  const double signA() const;

  /// 第二加速(減速)度方向の取得
  const double signD() const;

  /// 最大速度の取得
  const double v_max() const;

  /// 移動距離の取得
  const double xd() const;

  /// Step1,3移動時間の取得
  const double dT1() const;

  /// Step2移動時間の取得
  const double dT2() const;

  /// Step4移動時間の取得
  const double dT3() const;

  /// Step5,7移動時間の取得
  const double dT4() const;

  /// Step6移動時間の取得
  const double dT5() const;

  /// 合計時間の取得
  const double dT_total() const;

  /// Step1終端時刻＆Step2開始時刻の取得
  const double t1() const;

  /// Step1終端位置＆Step2開始位置の取得
  const double x1() const;

  /// Step1終端速度＆Step2開始速度の取得
  const double v1() const;

  /// Step2終端時刻＆Step3開始時刻の取得
  const double t2() const;

  /// Step2終端位置＆Step3開始位置の取得
  const double x2() const;

  /// Step2終端速度＆Step3開始速度の取得
  const double v2() const;

  /// Step3終端時刻＆Step4開始時刻の取得
  const double t3() const;

  /// Step3終端位置＆Step4開始位置の取得
  const double x3() const;

  /// Step3終端速度＆Step4開始速度の取得
  const double v3() const;

  /// Step4終端時刻＆Step5開始時刻の取得
  const double t4() const;

  /// Step4終端位置＆Step5開始位置の取得
  const double x4() const;

  /// Step4終端速度＆Step5開始速度の取得
  const double v4() const;

  /// Step5終端時刻＆Step6開始時刻の取得
  const double t5() const;

  /// Step5終端位置＆Step6開始位置の取得
  const double x5() const;

  /// Step5終端速度＆Step6開始速度の取得
  const double v5() const;

  /// Step6終端時刻＆Step7開始時刻の取得
  const double t6() const;

  /// Step6終端位置＆Step7開始位置の取得
  const double x6() const;

  /// Step6終端速度＆Step7開始速度の取得
  const double v6() const;

  /// Step7終端時刻の取得
  const double t7() const;

  /// 最速軌道の最大速度の取得
  const double v_max_fastest() const;

  /// 最速軌道のdT3の取得
  const double dT3_fastest() const;

  /// 最速軌道の最短時間の取得
  const double tf_fastest() const;

  /// 初期化済み(構成パラメータ設定済み)フラグ(済みならtrue)の取得
  const bool is_initialized() const;

  /// 軌道生成済みフラグ(生成済みならtrue)の取得
  const bool is_generated() const;

  /// 移動なしフラグ(速度0のまま同じ位置に停止していたらtrue)の取得
  const bool no_movement() const;

  /// 最速軌道のフラグ(最速軌道ならtrue)の取得
  const bool is_fastest() const;

};

/////////////////////////////////////////////////////////////////////////////////////////

/// 台形型5251525次軌道の補間器のキュー
typedef std::deque<Trapezoid5251525> Trapezoid5251525_Queue;

}

#endif // _TRAJECTORY_GENERATOR_HPP_
