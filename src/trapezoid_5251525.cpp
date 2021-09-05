#include "trapezoid_5251525.hpp"

// #define DEBUG_ 1

#define V_EPSILON 1.0e-15
#define X_EPSILON 1.0e-13
#define T_EPSILON 1.0e-12

#define SIGNV(a) ((a>=0) ? 1: -1)

using namespace interp;

Trapezoid5251525::Trapezoid5251525 () :
  a_limit_(1200),
  d_limit_(1200),
  ratio_acc_dec_(1.0),
  a_lower_limit_ (a_limit_ * ratio_acc_dec_),
  d_lower_limit_ (d_limit_ * ratio_acc_dec_),
  v_limit_(170),
  asr_(0.0),
  dsr_(0.0),
  is_initialized_(false),
  is_generated_  (false),
  no_movement_   (false),
  is_fastest_    (false) {
}

Trapezoid5251525::Trapezoid5251525 (const double& a_limit,
                                    const double& d_limit,
                                    const double& v_limit,
                                    const double& asr,
                                    const double& dsr,
                                    const double& ratio_acc_dec) {
  initialize(a_limit,
             d_limit,
             v_limit,
             asr,
             dsr,
             ratio_acc_dec);
}

Trapezoid5251525::Trapezoid5251525(const Trapezoid5251525& src) :
  x0_( src.x0() ),
  v0_( src.v0() ),
  xf_( src.xf() ),
  vf_( src.vf() ),
  t0_( src.t0() ),
  tf_( src.tf() ),
  a_limit_      ( src.a_limit()       ),
  d_limit_      ( src.d_limit()       ),
  ratio_acc_dec_( src.ratio_acc_dec() ),
  a_lower_limit_( src.a_lower_limit() ),
  d_lower_limit_( src.d_lower_limit() ),
  v_limit_      ( src.v_limit()       ),
  asr_          ( src.asr()           ),
  dsr_          ( src.dsr()           ),
  a_max_( src.a_max() ),
  d_max_( src.d_max() ),
  sign_ ( src.sign()  ),
  signA_( src.signA() ),
  signD_( src.signD() ),
  v_max_( src.v_max() ),
  xd_ ( src.xd()  ),
  dT1_( src.dT1() ),
  dT2_( src.dT2() ),
  dT3_( src.dT3() ),
  dT4_( src.dT4() ),
  dT5_( src.dT5() ),
  dT_total_( src.dT_total() ),
  t1_( src.t1() ),
  x1_( src.x1() ),
  v1_( src.v1() ),
  t2_( src.t2() ),
  x2_( src.x2() ),
  v2_( src.v2() ),
  t3_( src.t3() ),
  x3_( src.x3() ),
  v3_( src.v3() ),
  t4_( src.t4() ),
  x4_( src.x4() ),
  v4_( src.v4() ),
  t5_( src.t5() ),
  x5_( src.x5() ),
  v5_( src.v5() ),
  t6_( src.t6() ),
  x6_( src.x6() ),
  v6_( src.v6() ),
  t7_( src.t7() ),
  v_max_fastest_  ( src.v_max_fastest()  ),
  dT3_fastest_    ( src.dT3_fastest()    ),
  tf_fastest_     ( src.tf_fastest()     ),
  is_initialized_ ( src.is_initialized() ),
  is_generated_   ( src.is_generated()   ),
  no_movement_    ( src.no_movement()    ),
  is_fastest_     ( src.is_fastest()     ) {
}

Trapezoid5251525 Trapezoid5251525::operator=(const Trapezoid5251525& src) {
  Trapezoid5251525 dest(src);
  this->x0_ = dest.x0();
  this->v0_ = dest.v0();
  this->xf_ = dest.xf();
  this->vf_ = dest.vf();
  this->t0_ = dest.t0();
  this->tf_ = dest.tf();
  this->a_limit_       = dest.a_limit();
  this->d_limit_       = dest.d_limit();
  this->ratio_acc_dec_ = dest.ratio_acc_dec();
  this->a_lower_limit_ = dest.a_lower_limit();
  this->d_lower_limit_ = dest.d_lower_limit();
  this->v_limit_       = dest.v_limit();
  this->asr_           = dest.asr();
  this->dsr_           = dest.dsr();
  this->a_max_ = dest.a_max();
  this->d_max_ = dest.d_max();
  this->sign_  = dest.sign();
  this->signA_ = dest.signA();
  this->signD_ = dest.signD();
  this->v_max_ = dest.v_max();
  this->xd_  = dest.xd();
  this->dT1_ = dest.dT1();
  this->dT2_ = dest.dT2();
  this->dT3_ = dest.dT3();
  this->dT4_ = dest.dT4();
  this->dT5_ = dest.dT5();
  this->dT_total_ = dest.dT_total();
  this->t1_ = dest.t1();
  this->x1_ = dest.x1();
  this->v1_ = dest.v1();
  this->t2_ = dest.t2();
  this->x2_ = dest.x2();
  this->v2_ = dest.v2();
  this->t3_ = dest.t3();
  this->x3_ = dest.x3();
  this->v3_ = dest.v3();
  this->t4_ = dest.t4();
  this->x4_ = dest.x4();
  this->v4_ = dest.v4();
  this->t5_ = dest.t5();
  this->x5_ = dest.x5();
  this->v5_ = dest.v5();
  this->t6_ = dest.t6();
  this->x6_ = dest.x6();
  this->v6_ = dest.v6();
  this->t7_ = dest.t7();
  this->v_max_fastest_  = dest.v_max_fastest();
  this->dT3_fastest_    = dest.dT3_fastest();
  this->tf_fastest_     = dest.tf_fastest();
  this->is_initialized_ = dest.is_initialized();
  this->is_generated_   = dest.is_generated();
  this->no_movement_    = dest.no_movement();
  this->is_fastest_     = dest.is_fastest();
  return *this;
}

void Trapezoid5251525::initialize(const double& a_limit,
                                  const double& d_limit,
                                  const double& v_limit,
                                  const double& asr,
                                  const double& dsr,
                                  const double& ratio_acc_dec) {
  a_limit_       = a_limit;
  d_limit_       = d_limit;
  ratio_acc_dec_ = ratio_acc_dec;
  a_lower_limit_ = a_limit * ratio_acc_dec;
  d_lower_limit_ = d_limit * ratio_acc_dec;
  v_limit_       = v_limit;
  asr_           = asr;
  dsr_           = dsr;
  //
  if(v_limit_ <= 0.0) {
    // 速度リミットが0.0以下(負)ならばエラー
    std::stringstream ss;
    ss << "v_limit_ must be positive. v_limit_: "
       << std::fixed << std::setprecision(15)
       << v_limit_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  if(a_limit_ <= 0.0) {
    // 加速度リミットが0.0以下(負)ならばエラー
    std::stringstream ss;
    ss << "a_limit_ must be positive. a_limit_: "
       << std::fixed << std::setprecision(15)
       << a_limit_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  if(d_limit_ <= 0.0) {
    // 減速度リミットが0.0以下(負)ならばエラー
    std::stringstream ss;
    ss << "d_limit_ must be positive. d_limit_: "
       << std::fixed << std::setprecision(15)
       << d_limit_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }

  if(ratio_acc_dec_ < 0.0 || 1.0 < ratio_acc_dec_) {
    // 下限値の比率が0.0〜1.0の範囲を超えていればエラー
    std::stringstream ss;
    ss << "ratio_acc_dec must be from 0,0 to 1.0. ratio_acc_dec: "
       << std::fixed << std::setprecision(15)
       << ratio_acc_dec_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  //
  is_initialized_= true;
  //
  // 軌道構成パラメータを初期化したため,軌道未生成状態へリセット.
  // pop()を禁止させる
  is_generated_ = false;
  no_movement_  = false;
  is_fastest_   = false;
}

double Trapezoid5251525::generate_path(const double& ts, const double& tf,
                                       const double& xs, const double& xf,
                                       const double& vs, const double& vf ) {
  // if ( !is_initialized_ ) {
  //   // 軌道構成パラメータが与えられてない
  //   throw std::runtime_error("Not initialized config parameter yet.");
  // }

  is_generated_ = false;
  no_movement_  = false;
  is_fastest_   = false;
  t0_ = ts;
  tf_ = tf;
  x0_ = xs;
  xf_ = xf;
  v0_ = vs;
  vf_ = vf;

  // 0. 入力チェック
  this->input_check();

  // 移動なしフラグが立っていれば軌道を生成しない
  if (no_movement_) {
    // 最終到達時刻を終了時間にしておく
    t7_ = tf_;
    // 軌道生成完了フラグを立てる
    this->is_generated_ = true;
    return -1.0;
  }

  // 1. 最大速度方向の初期設定
  double sign_init = calc_initial_v_max_direction_sign();

#ifdef DEBUG_
  std::cout << std::fixed << std::setprecision(15);
  std::cout << "\n ==== Tragectory Generator ====\n";
  std::cout << "accleration_limit: " << a_limit_ << std::endl;
  std::cout << "decceleration_limit: " << d_limit_ << std::endl;
  std::cout << "v_limit: " << v_limit_ << std::endl;
  std::cout << "a_smoothing_rate: " << asr_ << std::endl;
  std::cout << "d_smoothing_rate: " << dsr_ << std::endl;
  std::cout << "sign : " << sign_ << std::endl;
  std::cout << "t0 : " << t0_  << std::endl;
  std::cout << "x0 : " << x0_  << std::endl;
  std::cout << "v0 : " << v0_  << std::endl;
  std::cout << "tf : " << tf_  << std::endl;
  std::cout << "xf : " << xf_  << std::endl;
  std::cout << "vf : " << vf_  << std::endl;
  std::cout << std::endl;
#endif

  // 2. 加速度上限値による、最速軌道の最大速度と最短時間の算出
  this->calc_fastest_parameter(a_limit_, d_limit_);
  a_max_ = a_limit_;
  d_max_ = d_limit_;

  // 3. 到達限界・速度反転領域の判定
  this->judge_reach_limitation();
  signA_ = sign_;
  signD_ = sign_;
#ifdef DEBUG_
    std::cout << "sign on v_max_fastest : " << sign_ << std::endl;
#endif

  // 最速軌道の最大速度方向が反転していた場合
  if ( sign_ != sign_init ) {
#ifdef DEBUG_
    std::cout << "recalculate v_max_fastest." << std::endl;
#endif

    // 反転符号で再度最速軌道の最大速度を再計算
    this->calc_fastest_parameter(a_limit_, d_limit_);
  }


  // 最終到達時刻が与えられている場合 = 最速軌道でない場合
  // 最速軌道でも、上で時間＆パラメータを求めた後、以降4, 5の計算で再度パラメータ値を確定させる
  {
    // 加速度を、最短時間に対する目標移動時間の比率により、上限値と下限値の内分をとって算出
    calc_acceleration_with_ratio();

#ifdef DEBUG_
    std::cout << "a_max_ : " << a_max_ << std::endl;
    std::cout << "d_max_ : " << d_max_ << std::endl;
#endif

    // 新しい加速度による、最速軌道の最大速度と最短時間の算出
    this->calc_fastest_parameter(a_max_, d_max_);
    // 新しい加速度による、到達限界・速度反転領域の判定
    this->judge_reach_limitation();
    signA_ = sign_;
    signD_ = sign_;
#ifdef DEBUG_
    std::cout << "sign : " << sign_ << std::endl;
#endif
    // 4. 移動距離の再計算
    xd_ = xf_ - x0_;

    // 5. 最大速度v_maxと等速移動時間dT3の算出
    this->calc_v_max_and_dT3();
  }

  // 6. 軌道パラメータを算出する
  this->set_parameter();

  // 7, 軌道生成完了フラグを立てる
  this->is_generated_ = true;

  return dT_total_;
}

/// 0.
void Trapezoid5251525::input_check() {
  if(t0_ < 0.0 || tf_ < 0.0){
    // 開始時刻もしくは終了時刻が負の場合エラー
    std::stringstream ss;
    ss << "t0 < 0.0 or tf < 0.0\n : t0 = "
       << std::fixed << std::setprecision(15)
       << t0_ << "\n   tf = " << tf_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  if (tf_ != 0.0 && t0_ > tf_) {
    // 開始時刻が終端時刻を超えていたらエラー
    std::stringstream ss;
    ss << "t0 exceeds tf (t0 > tf) : "
       << std::fixed << std::setprecision(15)
       << t0_ << " > " << tf_;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  if((fabs(v0_) > v_limit_) || (fabs(vf_) > v_limit_)) {
    // 初期速度、終端速度が速度リミットを超えていたらエラー
    std::stringstream ss;
    ss << "|v0| or |v1| ("
       << std::fixed << std::setprecision(15)
       << fabs(v0_) << " or " << fabs(vf_)
       << ") exceeds v_limit (" << v_limit_ <<  ")";
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument( ss.str() );
  }
  // 開始と終了地点が同じでかつ開始と終了の速度0.0ならば移動なしフラグを立てる
  if ( fabs(x0_-xf_)<= X_EPSILON && fabs(v0_)<= V_EPSILON && fabs(vf_)<= V_EPSILON) {
      no_movement_ = true;
#ifdef DEBUG_
      std::cout << "no_movement_ : " << no_movement_ << std::endl;
#endif
  }
  // 終了時刻が0.0の場合、最速軌道フラグを立てる
  if( tf_ == 0.0 )
  {
    is_fastest_ = true;
  }

}

/// 1.
double Trapezoid5251525::calc_initial_v_max_direction_sign() {
  sign_ = (xf_ - x0_ > 0) ? 1 : -1;
  // 位置が同じ場合,
  if (fabs(x0_-xf_) <= X_EPSILON) {
    sign_ = ( fabs(v0_) < fabs(vf_) ) ? SIGNV(vf_) : SIGNV(v0_);
  }
  return sign_;
}

/// 2.
void Trapezoid5251525::calc_fastest_parameter( const double& a_max,
                                               const double& d_max ) {
  // 移動距離
  xd_ = sign_ * (xf_-x0_);

  // 三角形軌道の最高速度を算出
  double p1 = a_max*(1+asr_);
  double p2 = d_max*(1+dsr_);
  double p3 =  (p1*v0_*v0_+ p2*vf_*vf_+ 2*a_max*d_max*xd_) / (p1 + p2);
  v_max_fastest_ = sign_*std::sqrt(fabs(p3));
#ifdef DEBUG_
  std::cout << "p3 : " << p3 << std::endl;
#endif
  // 境界台形
  double xd_limit =
    0.5*(1+asr_)*(v_limit_*v_limit_ - v0_*v0_)/a_max
    +
    0.5*(1+dsr_)*(v_limit_*v_limit_ - vf_*vf_)/d_max;
#ifdef DEBUG_
  std::cout << "xd_: " << xd_ << std::endl;
  std::cout << "xd_limit: " << xd_limit << std::endl;
#endif

  // 最速軌道が台形か三角形かチェック
  if( fabs(v_max_fastest_) > v_limit_) {
    // 台形
    v_max_fastest_ = sign_ * v_limit_;
    dT3_fastest_   = fabs( (xd_limit - xd_) / v_limit_ );
  } else {
    // 三角形
    dT3_fastest_ = 0.0;
  }

  double dT1_fastest = asr_*fabs(v_max_fastest_ - v0_) / a_max;
  double dT2_fastest = (1-asr_)*fabs(v_max_fastest_ - v0_) / a_max;
  double dT4_fastest = dsr_*fabs(v_max_fastest_ - vf_) / d_max;
  double dT5_fastest = (1-dsr_)*fabs(v_max_fastest_ - vf_) / d_max;
  tf_fastest_ = t0_ + 2*dT1_fastest + dT2_fastest + dT3_fastest_ + 2*dT4_fastest + dT5_fastest;

#ifdef DEBUG_
  std::cout << "is_fastest_ : " << is_fastest_ << std::endl;
  std::cout << "v_max_fastest : " << v_max_fastest_ << std::endl;
  std::cout << "dT3_fastest : " << dT3_fastest_ << std::endl;
  std::cout << "tf_fastest : " << tf_fastest_ << std::endl;
#endif

  // tf_の時刻がtf_fastest_とほぼ同じ場合、最速軌道動作とする
  if ( (is_fastest_)
       || ((tf_ - tf_fastest_ >= 0.0) && (tf_ - tf_fastest_ <= T_EPSILON))
       || (fabs(tf_ - tf_fastest_) <= X_EPSILON)
     ) {

    tf_    = (tf_ < tf_fastest_) ? tf_fastest_ : tf_;
    v_max_ = v_max_fastest_;
    dT3_   = dT3_fastest_;
    is_fastest_ = true;

  } else if ( tf_ < tf_fastest_ ) {
    std::stringstream ss;
    ss << "unreachable parameter. tf < fastest time: "
       << std::fixed << std::setprecision(15)
       << tf_ << " < " << tf_fastest_;
    std::cerr << ss.str() << std::endl;
    // 到達不可能！エラー！
    throw std::invalid_argument( ss.str() );
  }

}

/// 3.
void Trapezoid5251525::judge_reach_limitation() {
  // 到達不能領域L1識別子：v_step2_square_x_eq_xfの算出
  double v_max_L1 = v_max_fastest_;
  // double v_max_L1 = v_limit_;
  double dT1_L1   = asr_ * fabs(v_max_L1 - v0_) / a_max_;
  double x1_L1    = 0.15*sign_*a_max_*dT1_L1*dT1_L1 + v0_*dT1_L1 + x0_;
  double v1_L1    = 0.50*sign_*a_max_*dT1_L1 + v0_;
  double v_step2_square_x_eq_xf = fabs(sign_*2.0*a_max_*(xf_ - x1_L1) + v1_L1*v1_L1);
#ifdef DEBUG_
  std::cout << "v_max_L1 : " << v_max_L1 << std::endl;
  std::cout << "v_step2_square_x_eq_xf : " << v_step2_square_x_eq_xf << std::endl;
#endif

  // 到達不能領域L2識別子：x_step6_x_eq_0,
  // 到達不能領域L3識別子：v_step6_square_x_eq_xfの算出
  double v_max_L2L3 = v0_;
  double xf_L2L3    = x0_;
  double vf_L2L3    = (-1.0) * v0_;
  double sign_L2L3   = (v0_ >= 0.0) ? 1 : -1;
  if(v0_ == 0.0) {
    sign_L2L3 = (vf_>= 0.0) ? -1 : 1;
  }
#ifdef DEBUG_
  std::cout << "sign_L2L3 : " << sign_L2L3 << std::endl;
#endif
  double dT4_L2L3 = dsr_*fabs(v_max_L2L3 - vf_L2L3) / d_max_;
  double dT5_L2L3 = (1-dsr_)*fabs(v_max_L2L3 - vf_L2L3) / d_max_;
  double v6_L2L3  = vf_L2L3 + 0.50*sign_L2L3*d_max_*dT4_L2L3;
  double x6_L2L3  = xf_L2L3 + 0.35*sign_L2L3*d_max_*dT4_L2L3*dT4_L2L3 - v6_L2L3*dT4_L2L3;
  double v5_L2L3  = v6_L2L3 + sign_L2L3*d_max_*dT5_L2L3;
  double x5_L2L3  = x6_L2L3 + 0.50*sign_L2L3*d_max_*dT5_L2L3*dT5_L2L3 - v5_L2L3*dT5_L2L3;
  double x_step6_v_eq_0 = fabs(0.50*sign_L2L3*v5_L2L3*v5_L2L3 / d_max_ + x5_L2L3);
#ifdef DEBUG_
  std::cout << "x_step6_v_eq_0 : " << x_step6_v_eq_0 << std::endl;
#endif
  double v_step6_square_x_eq_xf = 2.0*sign_L2L3*d_max_*(x5_L2L3 - xf_)
                                   + v5_L2L3*v5_L2L3;
#ifdef DEBUG_
  std::cout <<  "v_step6_square_x_eq_xf : " << v_step6_square_x_eq_xf << std::endl;
  std::cout <<  "vf^2 : " << vf_*vf_ << std::endl;
#endif

  // 判定

#ifdef DEBUG_ // デバッグ出力
  std::cout << std::endl;
  std::cout << "in L1? " << std::endl;
  std::cout << "- v0_*vf(=" << (v0_*vf_) << ") >= 0.0 ? : "
            << (v0_*vf_ >= 0.0)
            << std::endl;
  std::cout << "- vf_*vf_(=" << (vf_*vf_)
            << ") >= v_step2_square_x_eq_xf(="
            << v_step2_square_x_eq_xf << ") ? : "
            << ( vf_*vf_ >= v_step6_square_x_eq_xf )
            << std::endl;
  std::cout << "- v_step2_square_x_eq_xf(="
            << v_step2_square_x_eq_xf
            << ") > v_max_L1*v_max_L1(="
            << v_max_L1*v_max_L1 << ") ? : "
            << ( v_step2_square_x_eq_xf > v_max_L1*v_max_L1 )
            << std::endl;
  std::cout << "- fabs(vf_)(=" << fabs(vf_)
            << ") >= fabs(v_max_L1)(=" << fabs(v_max_L1) << ") ? : "
            << ( fabs(vf_) >= fabs(v_max_L1) )
            << std::endl;
  std::cout << "- ( v0_*vf_ >= 0.0" << std::endl;
  std::cout << "   && ( vf_*vf_ >= v_step2_square_x_eq_xf" << std::endl;
  std::cout << "        || ( v_step2_square_x_eq_xf > v_max_L1*v_max_L1" << std::endl;
  std::cout << "             && fabs(vf_) >= fabs(v_max_L1) ))" << std::endl;
  std::cout << "   && sign_*vf_ >=0 ) ? : "
            << ( v0_*vf_ >= 0.0
                 && ( vf_*vf_ >= v_step2_square_x_eq_xf
                      || ( v_step2_square_x_eq_xf > v_max_L1*v_max_L1
                           && fabs(vf_) >= fabs(v_max_L1) ) )
                 && sign_*vf_ >=0 )
            << std::endl;
  std::cout << std::endl;
  std::cout << "in L4?" << std::endl;
  std::cout << "- v0_*vf_(="<< v0_*vf_<< ") <= 0.0 ? : "
            << (v0_*vf_ <= 0.0)
            << std::endl;
  std::cout << "- ( vf_*vf_(=" << vf_*vf_
            << ") >= v_step6_square_x_eq_xf(="
            << v_step6_square_x_eq_xf
            << ") ) ? : " << ( vf_*vf_ >= v_step6_square_x_eq_xf )
            << std::endl;
  std::cout << "- fabs(vf_)(=" << fabs(vf_)
            << ") > fabs(v0_)(=" << fabs(v0_) << ") ? : "
            << ( fabs(vf_) > fabs(v0_) )
            << std::endl;
  std::cout << "- sign_*vf_(" << sign_*vf_ << ") >= 0.0 ? : "
            << ( sign_*vf_ >= 0.0 )
            << std::endl;
  std::cout << "- ( (v0_*vf_ <= 0.0)" << std::endl;
  std::cout << "   && ( vf_*vf_ >= v_step6_square_x_eq_xf )" << std::endl;
  std::cout << "   && ( fabs(vf_) > fabs(v0_) )" << std::endl;
  std::cout << "   && sign_*vf_ >= 0.0 ) ? : "
            << ( (v0_*vf_ <= 0.0)
                 && ( vf_*vf_ >= v_step6_square_x_eq_xf )
                 && ( fabs(vf_) > fabs(v0_) )
                 && sign_*vf_ >= 0.0 )
            << std::endl;
  std::cout << std::endl;
  std::cout << "in L2 or L3?" << std::endl;
  std::cout << "- v0_(=" << v0_ << ") != 0.0 ? :"
            << (v0_ != 0.0)
            << std::endl;
  std::cout << "- SIGNV(v0_)(=" << SIGNV(v0_)
            << ") * ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) )(="
            << ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) ) << ") >= 0 ? :"
            << ( SIGNV(v0_) * ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) ) >= 0 )
            << std::endl;
  std::cout << "- vf_*vf_(=" << vf_*vf_
            << ") <= v_step6_square_x_eq_xf(="
            << v_step6_square_x_eq_xf << ") ? : "
            << (vf_*vf_<= v_step6_square_x_eq_xf)
            << std::endl;
  std::cout << "- xd_(=" << xd_
            << ") <= fabs(x_step6_v_eq_0 - x0_)(="
            << fabs(x_step6_v_eq_0 - x0_) << ") ? : "
            << (xd_ <= fabs(x_step6_v_eq_0 - x0_))
            << std::endl;
  std::cout << "- ( v0_ != 0.0" << std::endl;
  std::cout << "   && SIGNV(v0_) * ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) ) >= 0"
            << std::endl;
  std::cout << "   && vf_*vf_<= v_step6_square_x_eq_xf" << std::endl;
  std::cout << "   && xd_ <= fabs(x_step6_v_eq_0 - x0_ ) ? : ";
  std::cout << ( v0_ != 0.0
                 && SIGNV(v0_) * ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) ) >= 0
                 && vf_*vf_<= v_step6_square_x_eq_xf
                 && xd_ <= fabs(x_step6_v_eq_0 - x0_) )
            << std::endl;
  std::cout << std::endl;
#endif // ここまでデバッグ出力用

  if ( v0_*vf_ >= 0.0
       && ( vf_*vf_ >= v_step2_square_x_eq_xf
            || ( v_step2_square_x_eq_xf > v_max_L1*v_max_L1
                 && fabs(vf_) >= fabs(v_max_L1) ))
       && sign_*vf_ >=0 ) {
#ifdef DEBUG_
    std::cout << ">>>> (region L1 : vf^2 < v_step2_square_x_eq_xf) :"
              << (vf_*vf_) << " < " << v_step2_square_x_eq_xf << std::endl;
#endif
    // 到達不能領域L1
    // 速度を反転する
    sign_ = (-1) * sign_;

    // if (tf_ != 0.0) {
    //   // 終端時刻を指定した場合、加速度リミットエラーとする
    //   std::stringstream ss1, ss2;
    //   ss1 << (vf_*vf_);
    //   ss2 << v_step2_square_x_eq_xf;
    //   throw std::runtime_error( "unreachable parameter -- acceleratopn limit error.\n"
    //                             "  (region L1 : vf^2 < v_step2_square_x_eq_xf) :"
    //                             + ss1.str() + " < " + ss2.str() );
    // }

  } else if ( v0_*vf_ <= 0.0
              && ( vf_*vf_ >= v_step6_square_x_eq_xf )
              && ( fabs(vf_) > fabs(v0_) )
              && sign_*vf_ >= 0.0 ) {
#ifdef DEBUG_
    std::cout << ">>>> (region L4 : vf^2 >= v_step6_square_x_eq_xf) :"
              << (vf_*vf_) << " >= " << v_step6_square_x_eq_xf << std::endl;
#endif
    // // 到達不能領域L4
    sign_ = (-1) * sign_;

    // if (tf_ != 0.0) {
    //   // 終端時刻を指定した場合、加速度リミットエラーとする
    //   std::stringstream ss1, ss2;
    //   ss1 << (vf_*vf_);
    //   ss2 << v_step6_square_x_eq_xf;
    //   throw std::runtime_error( "unreachable parameter  -- acceleratopn limit error.\n"
    //                             "  (region L4 : vf^2 < v_step6_square_x_eq_xf) :"
    //                             + ss1.str() + " < " + ss2.str() );
    // }

  } else if ( v0_ != 0.0
              && SIGNV(v0_) * ( (xf_-x0_ > 0) ? 1 : ((xf_-x0_ == 0) ? 0 : -1) ) >= 0
              && vf_*vf_<= v_step6_square_x_eq_xf
              && xd_ <= fabs(x_step6_v_eq_0 - x0_) ) {
#ifdef DEBUG_
    std::cout << ">>>> (region L2,L3 : xd < |x_step6_v_eq_0 - x0|) :"
              << xd_ << " < " << fabs(x_step6_v_eq_0 - x0_) << std::endl;
#endif
    // 速度を反転する
    sign_ = (-1) * sign_;

    // if (tf_ != 0.0) {
    //   // 終端時刻を指定した場合、加速度リミットエラーとする
    //   std::stringstream ss1, ss2;
    //   ss1 << xd_;
    //   ss2 << fabs(x_step6_v_eq_0 - x0_);
    //   throw std::runtime_error( "unreachable parameter  -- acceleratopn limit error.\n"
    //                             "  (region L2,L3 : xd < |x_step6_v_eq_0 - x0|) :"
    //                             + ss1.str() + " < " + ss2.str() );
    // }
  }
}

///
void Trapezoid5251525::calc_acceleration_with_ratio() {
  // 前提として,ここでは
  // tf_ != 0.0 && tf_ >= t0_ >= 0.0
  // tf_ > tf_fastest_ が保証されている
  if (tf_ - t0_ == 0.0) {
    a_max_ = a_lower_limit_;
    d_max_ = d_lower_limit_;
    return;
  }
  double dt_ratio = (tf_fastest_ - t0_) / (tf_ - t0_);
  a_max_ = a_limit_*dt_ratio*dt_ratio + a_lower_limit_*(1.0 - dt_ratio*dt_ratio);
  d_max_ = d_limit_*dt_ratio*dt_ratio + d_lower_limit_*(1.0 - dt_ratio*dt_ratio);
}


/// 5.
double Trapezoid5251525::internal_calc_v_max_and_dT3(const double& signA,
                                                     const double& signD,
                                                     double& dT3, bool& ret) {
  double pA = signA*0.5*(1.0+asr_)/a_max_
            + signD*0.5*(1.0+dsr_)/d_max_;
  double pB = tf_-t0_ + signA*(1.0+asr_)*v0_/a_max_
                      + signD*(1.0+dsr_)*vf_/d_max_;
  double pC = xd_ + signA * 0.5*(1+asr_)*v0_*v0_/a_max_
                  + signD * 0.5*(1+dsr_)*vf_*vf_/d_max_;
  double pD = pB*pB - 4.0*pA*pC;
  double v_max;
  double xd = xd_;
  ret = true;
  // pA==0 && pB!=0
  if ( fabs(pA) < V_EPSILON && fabs(pB)>V_EPSILON ) {
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "Case 2,3" << std::endl;
#endif
    v_max = pC / pB;
    dT3 = (tf_ - t0_) - signA * (1.0+asr_)*(v_max - v0_)/a_max_
                      - signD * (1.0+dsr_)*(v_max - vf_)/d_max_;
  } else if (pD < 0.0 ) {
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "pD no solution" << std::endl;
#endif
    if(!is_fastest_) {
      // ２次方程式の判別式が0未満で最速軌道でなければ解なしで終了
      ret = false;
    }
    v_max = v_max_fastest_;
    dT3   = dT3_fastest_;

  } else {
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "Case 1,4" << std::endl;
#endif
    v_max = 0.5*( pB - std::sqrt(pD) )/pA;
    dT3 = tf_ - t0_ - signA * (1.0+asr_)*(v_max - v0_)/a_max_
                    - signD * (1.0+dsr_)*(v_max - vf_)/d_max_;
    if( is_fastest_
        && (fabs(v_max) > fabs(v_max_fastest_)) ) {
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "v_max(=" << v_max
              << ") > v_max_fastest(=" << v_max_fastest_
              << ")" << std::endl;
#endif
      v_max = v_max_fastest_;
      dT3   = dT3_fastest_;
    }
    xd = signA*0.5*(1.0+asr_)*(v_max*v_max - v0_*v0_)/a_max_
       + signD*0.5*(1.0+dsr_)*(v_max*v_max - vf_*vf_)/d_max_ + v_max*dT3 ;
  }
  // v_maxとv0_の大小関係, v_maxとvfの大小関係のパターンに
  // 当てはまらなければ解なしで終了
  if(sign_*v_max <= -1.0 * V_EPSILON
     || signA*(v_max - v0_) < 0.0
     || signD*(v_max - vf_) < 0.0
     || fabs(xd - xd_) > X_EPSILON
     || dT3 < 0.0) {
    ret = false;
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "not satisfied with all condition" << std::endl;
#endif
  }
#ifdef DEBUG_
  std::cout << std::endl;
  std::cout << "v0 : " << v0_ << std::endl;
  std::cout << "vf : " << vf_ << std::endl;
  std::cout << "sign : " << sign_ << std::endl;
  std::cout << "signA : " << signA << std::endl;
  std::cout << "signD : " << signD << std::endl;
  std::cout << "pA : " << pA << std::endl;
  std::cout << "pB : " << pB << std::endl;
  std::cout << "pC : " << pC << std::endl;
  std::cout << "pD : " << pD << std::endl;
  std::cout << "tf - t0 : " << tf_-t0_ << std::endl;
  std::cout << "v_max : " << v_max << std::endl;
  std::cout << "dT3 : " << dT3 << std::endl;
  std::cout << "xd_ : " << xd_ << std::endl;
  std::cout << "xd : " << xd << std::endl;
  std::cout << "sign_*v_max(="<<(sign_*v_max)<<") <= -"<<V_EPSILON<<" : "
            << (sign_*v_max < -1.0*V_EPSILON) << std::endl;
  std::cout << "signA*(v_max - v0_)(="<<(signA*(v_max - v0_))<<") < 0.0 : "
            << (signA*(v_max - v0_) < 0.0) << std::endl;
  std::cout << "signD*(v_max - vf_)(="<<(signD*(v_max - vf_))<<") < 0.0 : "
            << (signD*(v_max - vf_) < 0.0) << std::endl;
  std::cout << "fabs(xd - xd_)(="<<(fabs(xd - xd_))<< ") > "<< X_EPSILON<<" : "
            << (fabs(xd - xd_) > X_EPSILON) << std::endl;
  std::cout << "dT3 < 0.0 : " << (dT3 < 0.0) << std::endl;
#endif
  return v_max;
}

void Trapezoid5251525::calc_v_max_and_dT3() {
  bool ret;
  signA_ = sign_;
  signD_ = sign_;
  v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  if ( !ret ) {
    signA_ = (-1)*sign_;
    signD_ = sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }
  if ( !ret ) {
    signA_ = sign_;
    signD_ = (-1)*sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }
  if ( !ret ) {
    signA_ = (-1)*sign_;
    signD_ = (-1)*sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }

  ////

  if ( !ret ) {
    sign_ = (-1)*sign_;
    signA_ = sign_;
    signD_ = sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }
  if ( !ret ) {
    signA_ = (-1)*sign_;
    signD_ = sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }
  if ( !ret ) {
    signA_ = sign_;
    signD_ = (-1)*sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }
  if ( !ret ) {
    signA_ = (-1)*sign_;
    signD_ = (-1)*sign_;
    v_max_ = this->internal_calc_v_max_and_dT3(signA_, signD_, dT3_, ret);
  }

  ///

  if ( !ret ) {
    std::stringstream ss1;
    ss1 << std::fixed << std::setprecision(15);
    if (dT3_ < 0.0) {
      ss1 << "unreachable parameter. dT3 is not able to be solved.\n & ";
    }
    ss1 << "unreachable parameter. v_max is not able to be solved." << std::endl
        << "It seems that acceleration limit(a_max="<< a_max_ <<",d max=" << d_max_
        << ") could not be satisfied with inputs:" << std::endl
        << "   - time(=tf-t0)="       << tf_-t0_  << std::endl
        << "   - start position(x0)=" << x0_      << std::endl
        << "   - start_velocity(v0)=" << v0_      << std::endl
        << "   - goal_position(xf)="  << xf_      << std::endl
        << "   - goal_velocity(vf)="  << vf_      << std::endl
        << "   - v_limit: "           << v_limit_ << std::endl
        << "   - a_smoothing_rate: "  << asr_     << std::endl
        << "   - d_smoothing_rate: "  << dsr_     << std::endl;
    std::cerr << ss1.str() << std::endl;
    throw std::runtime_error( ss1.str() );
  }

  if (fabs(v_max_) > fabs(v_limit_)+V_EPSILON) {
    std::stringstream ss2;
    ss2 << std::fixed << std::setprecision(15)
        << "unreachable parameter. solved v_max(="
        << fabs(v_max_)
        << ") exceeds v_limit+" << V_EPSILON << "(="
        << (fabs(v_limit_)+V_EPSILON) << ").";
    std::cerr << ss2.str() << std::endl;
    throw std::runtime_error( ss2.str() );
  }
}

/// 6.
void Trapezoid5251525::set_parameter() {
  dT1_ = asr_ * fabs(v_max_ - v0_)/a_max_;
  dT2_ = (1-asr_) * fabs(v_max_ - v0_)/a_max_;
  dT4_ = dsr_ * fabs(v_max_ - vf_)/d_max_;
  dT5_ = (1-dsr_) * fabs(v_max_ - vf_)/d_max_;

  v1_ = signA_ * 0.50 * a_max_ * dT1_ + v0_;
  x1_ = signA_ * 0.15 * a_max_ * dT1_ * dT1_ + v0_ * dT1_ + x0_;
  v2_ = signA_ * a_max_ * dT2_ + v1_;
  x2_ = signA_ * 0.50 * a_max_ * dT2_ * dT2_ + v1_ * dT2_ + x1_;
  v3_ = signA_ * 0.50 * a_max_ * dT1_ + v2_;
  x3_ = signA_ * 0.35 * a_max_ * dT1_ * dT1_ + v2_ * dT1_ + x2_;

  v6_ = vf_ + signD_ * 0.50 * d_max_ * dT4_;
  x6_ = xf_ + signD_ * 0.35 * d_max_ * dT4_ * dT4_ - v6_ * dT4_;
  v5_ = v6_ + signD_ * d_max_ * dT5_;
  x5_ = x6_ + signD_ * 0.50 * d_max_ * dT5_ * dT5_ - v5_ * dT5_;
  v4_ = v_max_;
  x4_ = x5_ + signD_ * 0.15 * d_max_ * dT4_ * dT4_ - v_max_ * dT4_;
  /*"dT3";
    dT3 = cabs((x4-x3) / v_max_);*/
  t1_ = t0_ + dT1_;
  t2_ = t1_ + dT2_;
  t3_ = t2_ + dT1_;
  t4_ = t3_ + dT3_;
  t5_ = t4_ + dT4_;
  t6_ = t5_ + dT5_;
  t7_ = t6_ + dT4_;
  dT_total_ = 2*dT1_ + dT2_ + dT3_ + 2*dT4_ + dT5_;

#ifdef DEBUG_
  std::cout << "dT1 : " << dT1_ << std::endl;
  std::cout << "dT2 : " << dT2_ << std::endl;
  std::cout << "dT3 : " << dT3_ << std::endl;
  std::cout << "dT4 : " << dT4_ << std::endl;
  std::cout << "dT5 : " << dT5_ << std::endl;
  std::cout << "xd : " << xd_ << std::endl;
  std::cout << "v0 : " << v0_ << std::endl;
  std::cout << "x0 : " << x0_ << std::endl;
  std::cout << "v1 : " << v1_ << std::endl;
  std::cout << "x1 : " << x1_ << std::endl;
  std::cout << "v2 : " << v2_ << std::endl;
  std::cout << "x2 : " << x2_ << std::endl;
  std::cout << "v3 : " << v3_ << std::endl;
  std::cout << "x3 : " << x3_ << std::endl;
  std::cout << "v6 : " << v6_ << std::endl;
  std::cout << "x6 : " << x6_ << std::endl;
  std::cout << "v5 : " << v5_ << std::endl;
  std::cout << "x5 : " << x5_ << std::endl;
  std::cout << "v4 : " << v4_ << std::endl;
  std::cout << "x4 : " << x4_ << std::endl;
  std::cout << "t1 : " << t1_ << std::endl;
  std::cout << "t2 : " << t2_ << std::endl;
  std::cout << "t3 : " << t3_ << std::endl;
  std::cout << "t4 : " << t4_ << std::endl;
  std::cout << "t5 : " << t5_ << std::endl;
  std::cout << "t6 : " << t6_ << std::endl;
  std::cout << "t7 : " << t7_ << std::endl;
#endif

}

const int Trapezoid5251525::pop(const double& t, double& xt, double& vt, double& at) const {
  if (!is_generated_) {
    std::string err_msg = "Not generated path yet.";
    std::cerr << err_msg << std::endl;
    throw std::runtime_error( err_msg );
  }

  if (no_movement_) {
    xt = x0_;
    vt = v0_;
    at = 0.0;
    return 0;
  }

  if ( t0_ <= t && t < t1_) {
    // Step1
    xt = signA_ * (-0.1) * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t0_) * (t-t0_) * (t-t0_) * (t-t0_) * (t-t0_)
       + signA_ * 0.25 * a_max_/(dT1_ * dT1_)
          * (t-t0_) * (t-t0_) * (t-t0_) * (t-t0_)
       + v0_ * (t-t0_) + x0_;

    vt = signA_ * (-0.50) * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t0_) * (t-t0_) * (t-t0_) * (t-t0_)
       + signA_ * a_max_/(dT1_ * dT1_)
          * (t-t0_) * (t-t0_) * (t-t0_) + v0_;

    at = signA_ * (2.0) * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t0_) * (t-t0_) * (t-t0_)
       + signA_ * (3.0) * a_max_/(dT1_ * dT1_)
          * (t-t0_) * (t-t0_);

  } else if ( t1_ <= t && t < t2_) {
    // Step2
    xt = signA_ * 0.50 * a_max_ * (t-t1_) * (t-t1_) + v1_ * (t-t1_) + x1_;

    vt = signA_ * 1.00 * a_max_ * (t-t1_) + v1_;

    at = signA_ * a_max_;

  } else if ( t2_ <= t && t < t3_) {
    // Step3
    xt = signA_ * 0.10 * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t2_) * (t-t2_) * (t-t2_) * (t-t2_) * (t-t2_)
       - signA_ * (0.25) * a_max_/(dT1_ * dT1_)
          * (t-t2_) * (t-t2_) * (t-t2_) * (t-t2_)
       + signA_ * 0.50 * a_max_ * (t-t2_) * (t-t2_)
       + v2_ * (t-t2_) + x2_;

    vt = signA_ * 0.50 * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t2_) * (t-t2_) * (t-t2_) * (t-t2_)
       - signA_ * a_max_/(dT1_ * dT1_) * (t-t2_) * (t-t2_) * (t-t2_)
       + signA_ * a_max_ * (t-t2_) + v2_;

    at = signA_ * 2.0 * a_max_/(dT1_ * dT1_ * dT1_)
          * (t-t2_) * (t-t2_) * (t-t2_)
       - signA_ * 3.0 * a_max_/(dT1_ * dT1_) * (t-t2_) * (t-t2_)
       + signA_ * a_max_;

  } else if ( t3_ <= t && t < t4_) {
    // Step4
    xt = v_max_ * (t-t3_) + x3_;
    vt = v_max_;
    at = 0.0;
  } else if ( t4_ <= t && t < t5_) {
    // Step5
    xt = signD_ * 0.10 * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t4_) * (t-t4_) * (t-t4_) * (t-t4_) * (t-t4_)
       - signD_ * 0.25 * d_max_/(dT4_ * dT4_)
           * (t-t4_) * (t-t4_) * (t-t4_) * (t-t4_)
       + v_max_ * (t-t4_) + x4_;

    vt = signD_ * 0.50 * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t4_) * (t-t4_) * (t-t4_) * (t-t4_)
       - signD_ * d_max_/(dT4_ * dT4_)
          * (t-t4_) * (t-t4_) * (t-t4_) + v_max_;

    at = signD_ * 2.0 * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t4_) * (t-t4_) * (t-t4_)
       - signD_ * 3.0 * d_max_/(dT4_ * dT4_)
          * (t-t4_) * (t-t4_);

  } else if ( t5_ <= t && t < t6_) {
    // Step6
    xt = signD_ * (-0.5) * d_max_ * (t-t5_) * (t-t5_) + v5_ * (t-t5_) + x5_;

    vt = signD_ * (-d_max_) * (t-t5_) + v5_;

    at = signD_ * (-d_max_);

  } else if ( t6_ <= t && t <= t7_+T_EPSILON) {
    // Step7
    xt = signD_ * (-0.1) * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t6_) * (t-t6_) * (t-t6_) * (t-t6_) * (t-t6_)
       + signD_ * 0.25 * d_max_/(dT4_ * dT4_)
          * (t-t6_) * (t-t6_) * (t-t6_) * (t-t6_)
       - signD_ * 0.50 * d_max_ * (t-t6_) * (t-t6_)
       + v6_ * (t-t6_) + x6_;

    vt = signD_ *  (-0.5) * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t6_) * (t-t6_) * (t-t6_) * (t-t6_)
       + signD_ * d_max_/(dT4_ * dT4_)
          * (t-t6_) * (t-t6_) * (t-t6_)
       - signD_ * d_max_ * (t-t6_) + v6_;

    at = signD_ *  (-2.0) * d_max_/(dT4_ * dT4_ * dT4_)
          * (t-t6_) * (t-t6_) * (t-t6_)
       + signD_ *  (-3.0) * d_max_/(dT4_ * dT4_)
          * (t-t6_) * (t-t6_)
       - signD_ * d_max_;

  } else {
    std::stringstream ss1;
    ss1 << std::fixed << std::setprecision(15);
    ss1 << "time value is out of range between t0(=" << t0_
        << ") and tf(t7)+"<<T_EPSILON<<"(=" << t7_+T_EPSILON << ").";
    std::cerr << ss1.str() << std::endl;
    throw std::out_of_range( ss1.str() );
  }

  return 0;
}


const double Trapezoid5251525::finish_time() {
  return t7_;
}

const double Trapezoid5251525::x0() const { return x0_; }
const double Trapezoid5251525::v0() const { return v0_; }
const double Trapezoid5251525::xf() const { return xf_; }
const double Trapezoid5251525::vf() const { return vf_; }
const double Trapezoid5251525::t0() const { return t0_; }
const double Trapezoid5251525::tf() const { return tf_; }
const double Trapezoid5251525::a_limit() const { return a_limit_; }
const double Trapezoid5251525::d_limit() const { return d_limit_; }
const double Trapezoid5251525::ratio_acc_dec() const { return ratio_acc_dec_; }
const double Trapezoid5251525::a_lower_limit() const { return a_lower_limit_; }
const double Trapezoid5251525::d_lower_limit() const { return d_lower_limit_; }
const double Trapezoid5251525::v_limit() const { return v_limit_; }
const double Trapezoid5251525::asr() const { return asr_; }
const double Trapezoid5251525::dsr() const { return dsr_; }
const double Trapezoid5251525::a_max() const { return a_max_; }
const double Trapezoid5251525::d_max() const { return d_max_; }
const double Trapezoid5251525::sign()  const { return sign_; }
const double Trapezoid5251525::signA() const { return signA_; }
const double Trapezoid5251525::signD() const { return signD_; }
const double Trapezoid5251525::v_max() const { return v_max_; }
const double Trapezoid5251525::xd()  const { return xd_; }
const double Trapezoid5251525::dT1() const { return dT1_; }
const double Trapezoid5251525::dT2() const { return dT2_; }
const double Trapezoid5251525::dT3() const { return dT3_; }
const double Trapezoid5251525::dT4() const { return dT4_; }
const double Trapezoid5251525::dT5() const { return dT5_; }
const double Trapezoid5251525::dT_total() const { return dT_total_; }
const double Trapezoid5251525::t1() const { return t1_; }
const double Trapezoid5251525::x1() const { return x1_; }
const double Trapezoid5251525::v1() const { return v1_; }
const double Trapezoid5251525::t2() const { return t2_; }
const double Trapezoid5251525::x2() const { return x2_; }
const double Trapezoid5251525::v2() const { return v2_; }
const double Trapezoid5251525::t3() const { return t3_; }
const double Trapezoid5251525::x3() const { return x3_; }
const double Trapezoid5251525::v3() const { return v3_; }
const double Trapezoid5251525::t4() const { return t4_; }
const double Trapezoid5251525::x4() const { return x4_; }
const double Trapezoid5251525::v4() const { return v4_; }
const double Trapezoid5251525::t5() const { return t5_; }
const double Trapezoid5251525::x5() const { return x5_; }
const double Trapezoid5251525::v5() const { return v5_; }
const double Trapezoid5251525::t6() const { return t6_; }
const double Trapezoid5251525::x6() const { return x6_; }
const double Trapezoid5251525::v6() const { return v6_; }
const double Trapezoid5251525::t7() const { return t7_; }
const double Trapezoid5251525::v_max_fastest() const { return v_max_fastest_;   }
const double Trapezoid5251525::dT3_fastest()   const { return dT3_fastest_;     }
const double Trapezoid5251525::tf_fastest()    const { return tf_fastest_;      }
const bool Trapezoid5251525::is_initialized()  const { return is_initialized_;  }
const bool Trapezoid5251525::is_generated()    const { return is_generated_;    }
const bool Trapezoid5251525::no_movement()     const { return no_movement_;     }
const bool Trapezoid5251525::is_fastest()      const { return is_fastest_;      }

///////////////////////////////////////////////////////////////////////////////


