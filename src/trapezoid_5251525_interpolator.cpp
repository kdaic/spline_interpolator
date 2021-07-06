#include "trapezoid_5251525_interpolator.hpp"

using namespace interp;

///////////////////////////////////////////////////////////////////////////////

void TrapezoidalInterpolator::create_trapzd_trajectory_que()
{
  for( std::deque<TrapezoidConfig>::iterator trapzd_config_itr = trapzd_config_que_.begin();
       trapzd_config_itr != trapzd_config_que_.end();
       trapzd_config_itr++ )
  {
    Trapezoid5251525 trapzd( trapzd_config_itr->a_limit,
                             trapzd_config_itr->d_limit,
                             trapzd_config_itr->v_limit,
                             trapzd_config_itr->asr,
                             trapzd_config_itr->dsr,
                             trapzd_config_itr->ratio_acc_dec );
    //
    trapzd_trajectory_que_.push_back( trapzd );
  } // End of for i=0 -> trapzd_config_que_.size()
}


TrapezoidalInterpolator::TrapezoidalInterpolator () :
  SplineInterpolator() {
  is_v_limit_ = true;
}


TrapezoidalInterpolator::TrapezoidalInterpolator(
                           const TrapezoidalInterpolator& src ) :
  SplineInterpolator(     src.is_path_generated_,
                          src.is_v_limit_,
                          src.v_limit_,
                          src.target_tpva_queue_     ),
  trapzd_config_que_    ( src.trapzd_config_que_     ),
  trapzd_trajectory_que_( src.trapzd_trajectory_que_ ) {
}

TrapezoidalInterpolator::TrapezoidalInterpolator (
                         const TrapezoidConfigQueue& trapzd_config_que ) :
  SplineInterpolator(),
  trapzd_config_que_( trapzd_config_que ) {
  is_v_limit_ = true;

  create_trapzd_trajectory_que();
}

TrapezoidalInterpolator::TrapezoidalInterpolator (
                         const double& a_limit,
                         const double& d_limit,
                         const double& v_limit,
                         const double& asr,
                         const double& dsr,
                         const double& ratio_acc_dec) :
  SplineInterpolator() {
  is_v_limit_ = true;
  initialize( a_limit,
              d_limit,
              v_limit,
              asr,  dsr,
              ratio_acc_dec );
}


TrapezoidalInterpolator::~TrapezoidalInterpolator() {
}


TrapezoidalInterpolator& TrapezoidalInterpolator::operator=(
                           const TrapezoidalInterpolator& src ) {
  TrapezoidalInterpolator dest(src);
  is_path_generated_     = dest.is_path_generated_;
  is_v_limit_            = dest.is_v_limit_;
  v_limit_               = dest.v_limit_;
  target_tpva_queue_     = dest.target_tpva_queue_;
  trapzd_config_que_     = dest.trapzd_config_que_;
  trapzd_trajectory_que_ = dest.trapzd_trajectory_que_;
  return *this;
}


void TrapezoidalInterpolator::initialize(
                              const TrapezoidConfigQueue& trapzd_config_que ) {
  trapzd_config_que_.clear();
  trapzd_config_que_ = trapzd_config_que;
  trapzd_trajectory_que_.clear();
  create_trapzd_trajectory_que();
}


void TrapezoidalInterpolator::initialize(
                              const double& a_limit,
                              const double& d_limit,
                              const double& v_limit,
                              const double& asr,
                              const double& dsr,
                              const double& ratio_acc_dec ) {
  trapzd_config_que_.clear();
  TrapezoidConfig trapzd_config( a_limit,
                                 d_limit,
                                 v_limit,
                                 asr,  dsr,
                                 ratio_acc_dec );
  trapzd_config_que_.push_back( trapzd_config );
  trapzd_trajectory_que_.clear();
  create_trapzd_trajectory_que();
}


RetCode TrapezoidalInterpolator::generate_path (
                                 const TPQueue& target_tp_queue,
                                 const double vs, const double vf,
                                 const double as, const double af ) {
  if( trapzd_trajectory_que_.size() == 0 ) {
    return SPLINE_UNINITIALIZED_INTERPOLATOR;
  }

  std::size_t target_tp_queue_size = target_tp_queue.size();
  if( target_tp_queue_size < 2
      || ( (target_tp_queue_size-1) != trapzd_trajectory_que_.size() ) )
  {
    return SPLINE_INVALID_QUEUE_SIZE;
  }

  TPQueue target_TimePosition_queue = target_tp_queue;
  TimePosition target_tp;
  NonUniformRoundingSpline nurs;

  // 開始点を予め挿入しておく
  target_tp = target_TimePosition_queue.pop();
  nurs.push( target_tp.time,
             target_tp.value );
  //
  // 教示点キューをクリア
  target_tpva_queue_.clear();
  //
  TimePVA target_start( target_tp.time,
                        PosVelAcc( target_tp.value, vs, as ) );
  target_tpva_queue_.push( target_start );
  TimePVA target_goal;
  bool endflag = false;
  for( std::size_t trajectory_idx=0;
       (trajectory_idx < trapzd_trajectory_que_.size())
         && (endflag == false)
         && (target_TimePosition_queue.size() > 0);
       trajectory_idx++ ) {

    target_tp = target_TimePosition_queue.pop();
    // 目標時刻・位置から時刻・位置・速度の点を生成(3点分貯める)
    nurs.push( target_tp.time,
               target_tp.value );

    if( target_tp_queue_size == 2 ) {
      // 教示点キューの数が2個しかない場合はループしない
      target_goal.time  = target_tp.time;
      target_goal.P.pos = target_tp.value;
      target_goal.P.vel = vf;
      target_goal.P.acc = af;
      endflag = true;
    }
    else if ( nurs.size() >= 3 ){
      target_goal = nurs.pop();
    }
    else {
      // nursが３点未満の場合、不均一丸みスプラインの速度計算未実施のため
      // まだ軌道生成できるような(目標速度をもった)目標goal点はpopできないためcontinue
      trajectory_idx--; // 軌道を生成するまでカウントしない
      continue;
    }
    // popしたことでnurs.size()==2
    //
    target_tpva_queue_.push( target_goal );
    //
    Trapezoid5251525& ref_trapzd = trapzd_trajectory_que_.at( trajectory_idx );
    double dT_total = ref_trapzd.generate_path( target_start.time,  target_goal.time,
                                                target_start.P.pos, target_goal.P.pos,
                                                target_start.P.vel, target_goal.P.vel );
    if( dT_total < 0.0 )
    {
      return SPLINE_FAIL_TO_GENERATE_PATH;
    }
    //
    target_start = target_goal;

  } // End of for trajectory_idx=0 -> trapzd_trajectory_que_.size()

  return SPLINE_SUCCESS;
}


RetCode TrapezoidalInterpolator::generate_path(
                                 const TPVAQueue& target_tpva_queue ) {

  if( (target_tpva_queue.size()-1) != trapzd_trajectory_que_.size() )
  {
    return SPLINE_INVALID_QUEUE_SIZE;
  }
  TPVAQueue target_TimePVA_queue = target_tpva_queue;
  target_tpva_queue_             = target_tpva_queue;

  TimePVA target_start = target_TimePVA_queue.pop();
  TimePVA target_goal;

  for( std::size_t trajectory_idx=0;
       (trajectory_idx < trapzd_trajectory_que_.size())
         && (target_TimePVA_queue.size() > 0);
       trajectory_idx++ ) {

    target_goal = target_TimePVA_queue.pop();
    //
    Trapezoid5251525& ref_trapzd = trapzd_trajectory_que_.at( trajectory_idx );
    double dT_total = ref_trapzd.generate_path( target_start.time,  target_goal.time,
                                                target_start.P.pos, target_goal.P.pos,
                                                target_start.P.vel, target_goal.P.vel );
    if( dT_total < 0.0 )
    {
      return SPLINE_FAIL_TO_GENERATE_PATH;
    }
    //
    target_start = target_goal;
  }

  return SPLINE_SUCCESS;
}


RetCode TrapezoidalInterpolator::generate_path_from_pva(
                                 const double& xs, const double& xf,
                                 const double& vs, const double& vf,
                                 const double& as, const double& af ) {
  if( trapzd_trajectory_que_.size() == 0 ) {
    return SPLINE_UNINITIALIZED_INTERPOLATOR;
  }

  Trapezoid5251525& ref_trapzd = trapzd_trajectory_que_.at(0);
  double dT_total = ref_trapzd.generate_path( 0.0,  0.0,
                                              xs,   xf,
                                              vs,   vf  );
  if( dT_total < 0.0 )
  {
    return SPLINE_FAIL_TO_GENERATE_PATH;
  }

  target_tpva_queue_.clear();
  target_tpva_queue_.push( 0.0,      xs, vs, as );
  target_tpva_queue_.push( dT_total, xf, vf, af );
  return SPLINE_SUCCESS;
}


const TimePVA TrapezoidalInterpolator::pop( const double& t ) {
  double xt, vt, at;
  std::size_t trajectory_idx = 0;
  bool is_out_of_range       = true;

  for( std::size_t idx=0; idx<target_tpva_queue_.size() - 1; idx++) {

    if( target_tpva_queue_.get( idx ).time <= t
        && t <= target_tpva_queue_.get( idx+1 ).time ) {
      trajectory_idx = idx;
      is_out_of_range  = false;
    }
  }

  if( is_out_of_range )
  {
    std::stringstream ss1;
    std::size_t finish_index = target_tpva_queue_.size() - 1;
    ss1 << std::fixed << std::setprecision(15);
    ss1 << "time value = "
        << t
        << " is out of range between time of start index[0] t0(="
        << target_tpva_queue_.get( 0 ).time
        << ") and time of finish index["
        << finish_index
        << "] tf(="
        << target_tpva_queue_.get( finish_index ).time
        << ").";
    std::out_of_range( ss1.str() );
  }

  trapzd_trajectory_que_[trajectory_idx].pop( t, xt, vt, at );
  TimePVA ret( t, PosVelAcc( xt, vt, at ) );

  return ret;
}


