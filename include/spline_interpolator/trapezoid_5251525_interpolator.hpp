#ifndef INCLUDE_TRAPEZOID_5251525_INTEPOLATOR_HPP_
#define INCLUDE_TRAPEZOID_5251525_INTEPOLATOR_HPP_

#include "spline_interpolator.hpp"
#include "trapezoid_5251525.hpp"
#include "non_uniform_rounding_spline.hpp"

namespace interp {

class TrapezoidalInterpolator : public SplineInterpolator
{
public:
  /// デフォルトコンストラクタ
  TrapezoidalInterpolator();

  /// コピーコンストラクタ
  /// @param[in] src コピー元
  TrapezoidalInterpolator( const TrapezoidalInterpolator& src );

  /// コンストラクタ
  /// @param[in] trpzd_config_que 台形型5251525次軌道の構成データのキュー。
  ///                             補間点区間の数と同じサイズ。以下が構成要素。
  ///                             - a_limit 第一加速(減速)度上限値
  ///                             - d_limit 第二加速(減速)度上限値
  ///                             - v_limit 最大速度リミット
  ///                             - asr 第一丸め率
  ///                             - dsr 第二丸め率
  ///                             - ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率
  TrapezoidalInterpolator( const TrapezoidConfigQueue& trapzd_config_que );

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

  /// 代入演算子
  /// @param[in] src コピー元
  /// @return *this
  TrapezoidalInterpolator& operator=( const TrapezoidalInterpolator& src );

  /// 初期化
  /// @param[in] trpzd_config_que 台形型5251525次軌道の構成データのキュー。
  ///                             補間点区間の数と同じサイズ。以下が構成要素。
  ///                             - a_limit 第一加速(減速)度上限値
  ///                             - d_limit 第二加速(減速)度上限値
  ///                             - v_limit 最大速度リミット
  ///                             - asr 第一丸め率
  ///                             - dsr 第二丸め率
  ///                             - ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率
  void initialize( const TrapezoidConfigQueue& trapzd_config_que );

  /// 初期化
  /// @param[in] a_limit 第一加速(減速)度上限値の参照
  /// @param[in] d_limit 第二加速(減速)度上限値の参照
  /// @param[in] v_limit 最大速度リミットの参照
  /// @param[in] asr 第一丸め率の参照
  /// @param[in] dsr 第二丸め率の参照
  /// @param[in] ratio_acc_dec 第一＆第二加速度上限値に対する下限値の比率の参照
  void initialize( const double& a_limit,
                   const double& d_limit,
                   const double& v_limit,
                   const double& asr,
                   const double& dsr,
                   const double& ratio_acc_dec );


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

private:
  /// コンストラクタにてTrapzoidConfigデータからtrapzd_trajectory_que_を生成
  void create_trapzd_trajectory_que();

  /// 台形型5251525次軌道の構成データ
  TrapezoidConfigQueue trapzd_config_que_;

  /// 台形型5251525次軌道(＆補間器)
  Trapezoid5251525_Queue trapzd_trajectory_que_;
};


}

#endif // End of #define INCLUDE_TRAPEZOID_5251525_INTEPOLATOR_HPP_
