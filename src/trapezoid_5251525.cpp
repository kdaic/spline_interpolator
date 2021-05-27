#include "trapezoid_5251525.hpp"

// #define DEBUG_ 1
#define SIGNV(a) ((a>=0) ? 1: -1)

using namespace interp;


Trapezoid5251525::Trapezoid5251525 (const double& a_limit,
                                    const double& d_limit,
                                    const double& v_limit,
                                    const double& asr,
                                    const double& dsr,
                                    const double& ratio_acc_dec) :
  a_limit_(a_limit), d_limit_(d_limit),
  ratio_acc_dec_(ratio_acc_dec),
  a_lower_limit_ (a_limit * ratio_acc_dec),
  d_lower_limit_ (d_limit * ratio_acc_dec),
  v_limit_(v_limit),
  asr_(asr), dsr_(dsr),
  is_generated_(false), no_movement_(false) {

  if(v_limit_ <= 0.0) {
    // 速度リミットが0.0以下(負)ならばエラー
    std::stringstream ss;
    ss << v_limit_;
    throw std::invalid_argument("v_limit_ must be positive. v_limit_: " + ss.str());
  }
  if(ratio_acc_dec_ < 0.0 || 1.0 < ratio_acc_dec_) {
    // 下限値の比率が0.0〜1.0の範囲を超えていればエラー
    std::stringstream ss;
    ss << ratio_acc_dec_;
    throw std::invalid_argument("ratio_acc_dec must be from 0,0 to 1.0. ratio_acc_dec: "
                                + ss.str());
  }
}

double Trapezoid5251525::generate_path(const double& ts, const double& tf,
                                       const double& xs, const double& xf,
                                       const double& vs, const double& vf ) {
  no_movement_ = false;
  is_generated_ = false;
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
  if (tf_ != 0.0 ) {
    // 加速度を、最短時間に対する目標移動時間の比率により、上限値と下限値の内分をとって算出
    calc_acceleration_with_ratio();

    sign_ = sign_init;
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

#ifdef DEBUG_
    std::cout << "a_max_ : " << a_max_ << std::endl;
    std::cout << "d_max_ : " << d_max_ << std::endl;
#endif

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
    std::stringstream ss1, ss2;
    ss1 << t0_;
    ss2 << tf_;
    throw std::invalid_argument("t0 < 0.0 or tf < 0.0\n : t0 = "
                                + ss1.str() + "\n   tf = " + ss2.str());
  }
  if (tf_ != 0.0 && t0_ > tf_) {
    // 開始時刻が終端時刻を超えていたらエラー
    std::stringstream ss1, ss2;
    ss1 << t0_;
    ss2 << tf_;
    throw std::invalid_argument("t0 exceeds tf (t0 > tf) : " + ss1.str() + " > " + ss2.str());
  }
  if(fabs(v0_) > v_limit_ || fabs(vf_) > v_limit_) {
    // 初期速度、終端速度が速度リミットを超えていたらエラー
    std::stringstream ss1, ss2, ss3;
    ss1 << fabs(v0_);
    ss2 << fabs(vf_);
    ss3 << v_limit_;
    throw std::invalid_argument("|v0| or |v1| (" + ss1.str() + " or " + ss2.str()
                                + ") exceeds v_limit (" + ss3.str() +  ")");
  }
  // 開始と終了地点が同じでかつ開始と終了の速度0.0ならば移動なしフラグを立てる
  if ( fabs(x0_-xf_)<= 1e-12 && fabs(v0_)<= 1e-12 && fabs(vf_)<= 1e-12) {
      no_movement_ = true;
  }

}

/// 1.
double Trapezoid5251525::calc_initial_v_max_direction_sign() {
  sign_ = (xf_ - x0_ > 0) ? 1 : -1;
  // 位置が同じ場合,
  if (fabs(x0_-xf_) <= 1.0e-12) {
    if (tf_ == 0.0) {
      // 最速軌道ならば,
      // 最大速度を開始-終端速度の絶対値の大きい方の符号をとる(等しい場合は開始側の符号)
      sign_ = ( fabs(v0_) < fabs(vf_) ) ? SIGNV(vf_) : SIGNV(v0_);
    } else {
      // 時間指定ならば
      // 最大速度を開始-終端速度の絶対値の大きい方の符号の反転をとる
      // (実際時間指定の場合、
      //  calc_v_max_and_dT3()で最大4通りの方法で最大速度の方向を試すため、
      //  ここでの符号によらず解は求まる)
      sign_ = ( fabs(v0_) < fabs(vf_) ) ? -SIGNV(vf_) : -SIGNV(v0_);
    }
  }
  return sign_;
}

/// 2.
void Trapezoid5251525::calc_fastest_parameter(const double& a_max, const double& d_max)  {
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
  double dT3_fastest;

  // 最速軌道が台形か三角形かチェック
  if( fabs(v_max_fastest_) > v_limit_) {
    // 台形
    v_max_fastest_ = sign_ * v_limit_;
    dT3_fastest = fabs( (xd_limit - xd_) / v_limit_ );
  } else {
    // 三角形
    dT3_fastest = 0.0;
  }

  double dT1_fastest = asr_*fabs(v_max_fastest_ - v0_) / a_max;
  double dT2_fastest = (1-asr_)*fabs(v_max_fastest_ - v0_) / a_max;
  double dT4_fastest = dsr_*fabs(v_max_fastest_ - vf_) / d_max;
  double dT5_fastest = (1-dsr_)*fabs(v_max_fastest_ - vf_) / d_max;
  tf_fastest_ = t0_ + 2*dT1_fastest + dT2_fastest + dT3_fastest + 2*dT4_fastest + dT5_fastest;

#ifdef DEBUG_
  std::cout << "v_max_fastest : " << v_max_fastest_ << std::endl;
  std::cout << "dT3_fastest : " << dT3_fastest << std::endl;
  std::cout << "tf_fastest : " << tf_fastest_ << std::endl;
#endif

  // tf_の時刻がtf_fastest_とほぼ同じ場合、最速軌道動作とする
  if ( tf_ == 0.0
       || ((tf_ - tf_fastest_ >= 0.0) && (tf_ - tf_fastest_ <= 1.0e-15)) ) {
    tf_ = 0.0;
    v_max_ = v_max_fastest_;
    dT3_ = dT3_fastest;
  } else if ( tf_ < tf_fastest_ ) {
    std::stringstream ss1, ss2;
    ss1 << std::fixed << std::setprecision(15);
    ss2 << std::fixed << std::setprecision(15);
    ss1 << tf_;
    ss2 << tf_fastest_;
    // 到達不可能！エラー！
    throw std::invalid_argument( "unreachable parameter. tf < fastest time: "
                                 + ss1.str() + " < " + ss2.str() );
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
  std::cout << "in L1? v_step2_square_x_eq_xf : " << v_step2_square_x_eq_xf << std::endl;
#endif

  // 到達不能領域L2識別子：x_step6_x_eq_0,
  // 到達不能領域L3識別子：v_step6_square_x_eq_xfの算出
  double v_max_L2L3 = v0_;
  double xf_L2L3    = x0_;
  double vf_L2L3    = (-1.0) * v0_;
  double sign_L2L3   = (v0_ >= 0.0) ? 1 : -1;
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
  std::cout << "in L2 or L3? x_step6_v_eq_0 : " << x_step6_v_eq_0 << std::endl;
#endif
  double v_step6_square_x_eq_xf = 2.0*sign_L2L3*d_max_*(x5_L2L3 - xf_)
                                   + v5_L2L3*v5_L2L3;
#ifdef DEBUG_
  std::cout <<  "in L4? v_step6_square_x_eq_xf : " << v_step6_square_x_eq_xf << std::endl;
  std::cout <<  "vf^2 : " << vf_*vf_ << std::endl;
#endif

  // 判定
  if ( v0_*vf_ >= 0.0
       && ( vf_*vf_ >= v_step2_square_x_eq_xf
            || ( v_step2_square_x_eq_xf > v_max_L1*v_max_L1
                 && fabs(vf_) >= fabs(v_max_L1) ))
       && sign_*vf_ >=0 ) {
#ifdef DEBUG_
    std::cout << "(region L1 : vf^2 < v_step2_square_x_eq_xf) :"
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
    std::cout << "(region L4 : vf^2 < v_step6_square_x_eq_xf) :"
              << (vf_*vf_) << " < " << v_step6_square_x_eq_xf << std::endl;
#endif
    // // 到達不能領域L4
    if ( tf_ == 0.0 ) {
      // 最速軌道を指定した場合、速度を反転する
      sign_ = (-1) * sign_;
    }
    // else {
    //   // 終端時刻を指定した場合、加速度リミットエラーとする
    //   std::stringstream ss1, ss2;
    //   ss1 << (vf_*vf_);
    //   ss2 << v_step6_square_x_eq_xf;
    //   throw std::runtime_error( "unreachable parameter  -- acceleratopn limit error.\n"
    //                             "  (region L4 : vf^2 < v_step6_square_x_eq_xf) :"
    //                             + ss1.str() + " < " + ss2.str() );
    // }

  } else if ( v0_ != 0.0
              && vf_*vf_<= v_step6_square_x_eq_xf
              && xd_ <= fabs(x_step6_v_eq_0 - x0_) ) {
#ifdef DEBUG_
      std::cout << "(region L2,L3 : xd < |x_step6_v_eq_0 - x0|) :"
                << xd_ << " < " << fabs(x_step6_v_eq_0 - x0_) << std::endl;
#endif
    if ( tf_ == 0.0 ) {
      // 最速軌道を指定した場合、速度を反転する
      sign_ = (-1) * sign_;
    }
    // else {
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
  if ( pA==0.0 && pB!=0.0 ) {
    v_max = pC / pB;
    dT3 = (tf_ - t0_) - signA * (1.0+asr_)*(v_max - v0_)/a_max_
                      - signD * (1.0+dsr_)*(v_max - vf_)/d_max_;
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "Case 2,3" << std::endl;
#endif

  } else if (pD < 0.0) {
    // ２次方程式の判別式が0未満なら解なしで終了
    ret = false;
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "pD no solution" << std::endl;
#endif
  } else {
    v_max = 0.5*( pB - std::sqrt(pD) )/pA;
    dT3 = tf_ - t0_ - signA * (1.0+asr_)*(v_max - v0_)/a_max_
                    - signD * (1.0+dsr_)*(v_max - vf_)/d_max_;
    xd = signA*0.5*(1.0+asr_)*(v_max*v_max - v0_*v0_)/a_max_
       + signD*0.5*(1.0+dsr_)*(v_max*v_max - vf_*vf_)/d_max_ + v_max*dT3 ;
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "Case 1,4" << std::endl;
#endif
  }
  // v_maxとv0_の大小関係, v_maxとvfの大小関係のパターンに
  // 当てはまらなければ解なしで終了
  if(sign_*v_max <= -1.0e-15
     || signA*(v_max - v0_) < 0.0
     || signD*(v_max - vf_) < 0.0
     || fabs(xd - xd_) > 1.0e-12
     || dT3 < 0.0) {
    ret = false;
#ifdef DEBUG_
    std::cout << std::endl;
    std::cout << "not satisfied with all condition" << std::endl;
#endif
  }
#ifdef DEBUG_
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
  std::cout << "sign_*v_max < 0.0 : " << (sign_*v_max < 0.0) << std::endl;
  std::cout << "signA*(v_max - v0_) < 0.0 : " << (signA*(v_max - v0_) < 0.0) << std::endl;
  std::cout << "signD*(v_max - vf_) < 0.0 : " << (signD*(v_max - vf_) < 0.0) << std::endl;
  std::cout << "fabs(xd - xd_) > 1.0e-14 : " << (fabs(xd - xd_) > 1.0e-14) << std::endl;
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
  if ( !ret ) {
    std::stringstream ss1;
    ss1 << std::fixed << std::setprecision(15);
    if (dT3_ < 0.0) {
      ss1 << "unreachable parameter. dT3 is not able to be solved.\n & ";
    }
    ss1 << "unreachable parameter. v_max is not able to be solved." << std::endl
        << "It seems that acceleration limit(a_max="<< a_max_ <<",d max=" << d_max_
        << ") could not be satisfied with inputs:" << std::endl
        << "   - time(=tf-t0)="<< tf_-t0_ << std::endl
        << "   - start position(x0)=" << x0_ << std::endl
        << "   - start_velocity(v0)="<< v0_ << std::endl
        << "   - goal_position(xf)="<< xf_ << std::endl
        << "   - goal_velocity(vf)="<< vf_ << std::endl;
    throw std::runtime_error(ss1.str());
  }
  if (fabs(v_max_) > fabs(v_limit_)) {
    throw std::runtime_error("unreachable parameter. solved v_max exceeds v_limit.");
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
  std::cout << std::fixed << std::setprecision(8) << "dT1 : " << dT1_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "dT2 : " << dT2_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "dT3 : " << dT3_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "dT4 : " << dT4_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "dT5 : " << dT5_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "xd : " << xd_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v0 : " << v0_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x0 : " << x0_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v1 : " << v1_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x1 : " << x1_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v2 : " << v2_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x2 : " << x2_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v3 : " << v3_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x3 : " << x3_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v6 : " << v6_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x6 : " << x6_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v5 : " << v5_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x5 : " << x5_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "v4 : " << v4_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "x4 : " << x4_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t1 : " << t1_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t2 : " << t2_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t3 : " << t3_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t4 : " << t4_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t5 : " << t5_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t6 : " << t6_ << std::endl;
  std::cout << std::fixed << std::setprecision(8) << "t7 : " << t7_ << std::endl;
#endif

}

int Trapezoid5251525::pop(const double& t, double& xt, double& vt, double& at) {
  if (!is_generated_) {
    throw std::runtime_error("Not generated path yet.");
  }

  if (no_movement_) {
    xt = x0_;
    vt = v0_;
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

  } else if ( t6_ <= t && t <= t7_+1e-12) {
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
        << ") and tf(t7)+1e-12(=" << t7_+1e-12 << ").";
    std::out_of_range(ss1.str());
  }

  return 0;
}


const double Trapezoid5251525::finish_time() {
  return t7_;
}

const double Trapezoid5251525::v_limit() {
  return v_limit_;
}


///////////////////////////////////////////////////////////////////////////////

TrapezoidalInterpolator::TrapezoidalInterpolator(
                         const double& a_limit,
                         const double& d_limit,
                         const double& v_limit,
                         const double& asr,
                         const double& dsr,
                         const double& ratio_acc_dec) :
  SplineInterpolator() {
  is_v_limit_ = true;
}

TrapezoidalInterpolator::~TrapezoidalInterpolator() {
}


RetCode TrapezoidalInterpolator::generate_path(
                                 const TPQueue& target_tp_queue,
                                 const double vs, const double vf,
                                 const double as, const double af ) {

  return SPLINE_SUCCESS;
}

RetCode TrapezoidalInterpolator::generate_path(
                                 const TPVAQueue& target_tpva_queue ) {
  return SPLINE_SUCCESS;
}

RetCode TrapezoidalInterpolator::generate_path_from_pva(
                                 const double& xs, const double& xf,
                                 const double& vs, const double& vf,
                                 const double& as, const double& af ) {
  return SPLINE_SUCCESS;
}

const TimePVA TrapezoidalInterpolator::pop( const double& t ) {
  TimePVA ret( t, PosVelAcc(0, 0, 0) );

  return ret;
}




