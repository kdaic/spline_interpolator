#include <gtest/gtest.h>
#include "spline_interpolator.hpp"

using namespace interp;

TEST(SplineExceptionTest, THROW) {
  try {
    THROW (SplineException, "This is Message.");
  } catch(SplineException& pe) {
    EXPECT_TRUE(  std::string(pe.what()).find("[SplineException] This is Message.") );
    std::cout << pe.what() << std::endl;
  }
}

TEST(g_isNearlyEq, test) {
  EXPECT_THROW( g_isNearlyEq("a", "b"), InvalidTypeArgument );
  int a_array[] = {0,1,2};
  int b_array[] = {0,1,2};
  EXPECT_THROW( g_isNearlyEq(a_array, b_array), InvalidTypeArgument );
  // This cannot be compiled.
  // std::vector<double> a_vector(a_array, a_array+3);
  // std::vector<double> b_vector(b_array, b_array+3);
  // EXPECT_THROW( g_isNearlyEq(a_vector, b_vector), InvalidTypeArgument );
  int a_int = 0;
  EXPECT_TRUE( g_isNearlyEq(a_int, a_int) );
  EXPECT_FALSE( g_isNearlyEq(a_int, a_int+1) );
  EXPECT_FALSE( g_isNearlyEq( double(a_int), a_int+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyEq( double(a_int), a_int+PRECISION) );
  float a_float = 0.0;
  EXPECT_TRUE( g_isNearlyEq(a_float, a_float) );
  EXPECT_FALSE( g_isNearlyEq( double(a_float), a_float+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyEq( double(a_float), a_float+PRECISION ) );
  double a_double = 0.0;
  EXPECT_TRUE( g_isNearlyEq(a_double, a_double) );
  EXPECT_FALSE( g_isNearlyEq( double(a_double), a_double+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyEq( double(a_double), a_double+PRECISION ) );
}

TEST(g_isNearlyZero, test) {
  EXPECT_THROW( g_isNearlyZero('a'), InvalidTypeArgument );
  // This cannot be compiled.
  // EXPECT_THROW( g_isNearlyZero("a"), InvalidTypeArgument );
  // int a_array[] = {0,1,2};
  // EXPECT_THROW( g_isNearlyZero(a_array), InvalidTypeArgument );
  int a_int = 0;
  EXPECT_TRUE( g_isNearlyZero(a_int) );
  EXPECT_FALSE( g_isNearlyZero(a_int+1) );
  EXPECT_FALSE( g_isNearlyZero( a_int+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyZero( a_int+PRECISION) );
  float a_float = 0.0;
  EXPECT_TRUE( g_isNearlyZero(a_float) );
  EXPECT_FALSE( g_isNearlyZero( a_float+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyZero( a_float+PRECISION ) );
  double a_double = 0.0;
  EXPECT_TRUE( g_isNearlyZero(a_double) );
  EXPECT_FALSE( g_isNearlyZero( a_double+PRECISION*1.1 ) );
  EXPECT_TRUE( g_isNearlyZero( a_double+PRECISION ) );
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TimeValTest, constructor) {
  TimeVal<double> tp0;
  double time = 1.3;
  double value = 10.2;
  tp0.time = time;
  tp0.value = value;
  EXPECT_EQ( tp0.value, tp0.P );

  TimeVal<double> tp1(tp0);
  EXPECT_EQ( tp0.time,  tp1.time  );
  EXPECT_EQ( tp0.value, tp1.value );
  EXPECT_EQ( tp0.P,     tp1.P     );
  EXPECT_EQ( tp1.value, tp1.P     );

  TimeVal<double> tp2(time, value);
  EXPECT_EQ( tp0.time,  tp2.time  );
  EXPECT_EQ( tp0.value, tp2.value );
  EXPECT_EQ( tp0.P,     tp2.P     );
  EXPECT_EQ( tp2.value, tp2.P     );
}

TEST(TimeValTest, insert_operator) {
  TimeVal<double> tp0(3.1, -88.7);
  TimeVal<double> tp1;
  // pre-check
  EXPECT_NE( tp0.time,  tp1.time  );
  EXPECT_NE( tp0.value, tp1.value );
  EXPECT_NE( tp0.P,     tp1.P     );
  // copy
  tp1 = tp0;
  //
  EXPECT_EQ( tp0.time,  tp1.time  );
  EXPECT_EQ( tp0.value, tp1.value );
  EXPECT_EQ( tp0.P,     tp1.P     );
  // self insert
  tp1 = tp1;
  //
  EXPECT_EQ( tp0.time,  tp1.time  );
  EXPECT_EQ( tp0.value, tp1.value );
  EXPECT_EQ( tp0.P,     tp1.P     );
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(PosVelAccTest, constructor ) {
  double default_val = 0.0;
  PosVelAcc pva0;
  /// default value of pos,vel,acc is 0.0
  EXPECT_EQ(default_val, pva0.pos);
  EXPECT_EQ(default_val, pva0.vel);
  EXPECT_EQ(default_val, pva0.acc);
}

TEST(PosVelAccTest, copy_constructor ) {
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  PosVelAcc pva_src;
  pva_src.pos = pos0;
  pva_src.vel = vel0;
  pva_src.acc = acc0;
  // copy constructor
  PosVelAcc pva_dest(pva_src);
  //
  // check whether copy is succeeded.
  EXPECT_EQ(pva_src.pos, pva_dest.pos);
  EXPECT_EQ(pva_src.vel, pva_dest.vel);
  EXPECT_EQ(pva_src.acc, pva_dest.acc);
  // check independent allocation
  double pos1 = 3.21;
  double vel1 = 6.54;
  double acc1 = 9.87;
  pva_src.pos = pos1;
  pva_src.vel = vel1;
  pva_src.acc = acc1;
  EXPECT_NE(pva_src.pos, pva_dest.pos);
  EXPECT_NE(pva_src.vel, pva_dest.vel);
  EXPECT_NE(pva_src.acc, pva_dest.acc);
  // check copied dest is not changed if src is changed.
  EXPECT_EQ(pos0, pva_dest.pos);
  EXPECT_EQ(vel0, pva_dest.vel);
  EXPECT_EQ(acc0, pva_dest.acc);
}

TEST(PosVelAccTest, value_constructor ) {
  double default_val = 0.0;
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  //
  // position
  PosVelAcc pva0(pos0);
  // check position copy and check other is default value
  EXPECT_EQ(pos0,        pva0.pos);
  EXPECT_EQ(default_val, pva0.vel);
  EXPECT_EQ(default_val, pva0.acc);
  //
  // position, velocity
  PosVelAcc pva1(pos0, vel0);
  //
  EXPECT_EQ(pos0,        pva1.pos);
  EXPECT_EQ(vel0,        pva1.vel);
  EXPECT_EQ(default_val, pva1.acc);
  //
  // position, velocity, acceleration
  PosVelAcc pva2(pos0, vel0, acc0);
  //
  EXPECT_EQ(pos0, pva2.pos);
  EXPECT_EQ(vel0, pva2.vel);
  EXPECT_EQ(acc0, pva2.acc);
}

TEST(PosVelAccTest, insert_operator ) {
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  PosVelAcc pva_src;
  pva_src.pos = pos0;
  pva_src.vel = vel0;
  pva_src.acc = acc0;
  // copy operator
  PosVelAcc pva_dest = pva_src;
  //
  // check whether copy is succeeded.
  EXPECT_EQ(pva_src.pos, pva_dest.pos);
  EXPECT_EQ(pva_src.vel, pva_dest.vel);
  EXPECT_EQ(pva_src.acc, pva_dest.acc);
  // check indipendent allocation
  double pos1 = 3.21;
  double vel1 = 6.54;
  double acc1 = 9.87;
  pva_src.pos = pos1;
  pva_src.vel = vel1;
  pva_src.acc = acc1;
  EXPECT_NE(pva_src.pos, pva_dest.pos);
  EXPECT_NE(pva_src.vel, pva_dest.vel);
  EXPECT_NE(pva_src.acc, pva_dest.acc);
  // check copied dest is not changed if src is changed.
  EXPECT_EQ(pos0, pva_dest.pos);
  EXPECT_EQ(vel0, pva_dest.vel);
  EXPECT_EQ(acc0, pva_dest.acc);
  //
  // self insert
  pva_dest = pva_dest;
  EXPECT_EQ(pos0, pva_dest.pos);
  EXPECT_EQ(vel0, pva_dest.vel);
  EXPECT_EQ(acc0, pva_dest.acc);
  // continuous insert
  PosVelAcc pva_dest2;
  {
    PosVelAcc pva_src2(pos1, vel1, acc1);
    PosVelAcc pva_dest3 = pva_dest2 = pva_src2 = pva_src2;
    EXPECT_EQ(pos1, pva_src2.pos);
    EXPECT_EQ(vel1, pva_src2.vel);
    EXPECT_EQ(acc1, pva_src2.acc);
    EXPECT_EQ(pos1, pva_dest2.pos);
    EXPECT_EQ(vel1, pva_dest2.vel);
    EXPECT_EQ(acc1, pva_dest2.acc);
    EXPECT_EQ(pos1, pva_dest3.pos);
    EXPECT_EQ(vel1, pva_dest3.vel);
    EXPECT_EQ(acc1, pva_dest3.acc);
  }
  EXPECT_EQ(pos1, pva_dest2.pos);
  EXPECT_EQ(vel1, pva_dest2.vel);
  EXPECT_EQ(acc1, pva_dest2.acc);
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TimePVATest, constructor) {
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  //
  TimePVA tpva0;
  tpva0.P.pos = pos0;
  tpva0.P.vel = vel0;
  tpva0.P.acc = acc0;
  //
  EXPECT_EQ( tpva0.P.pos, tpva0.P.pos );
  EXPECT_EQ( tpva0.P.vel, tpva0.P.vel );
  EXPECT_EQ( tpva0.P.acc, tpva0.P.acc );
  //
  double pos1 = 3.21;
  double vel1 = 6.54;
  double acc1 = 9.87;
  //
  tpva0.P.pos = pos1;
  tpva0.P.vel = vel1;
  tpva0.P.acc = acc1;
  //
  EXPECT_EQ( tpva0.P.pos, tpva0.P.pos );
  EXPECT_EQ( tpva0.P.vel, tpva0.P.vel );
  EXPECT_EQ( tpva0.P.acc, tpva0.P.acc );
  //
  double time0 = 4.44;
  TimePVA tpva1( time0 );
  EXPECT_EQ( time0, tpva1.time );
  EXPECT_EQ( 0.0,   tpva1.P.pos );
  EXPECT_EQ( 0.0,   tpva1.P.vel );
  EXPECT_EQ( 0.0,   tpva1.P.acc );
  //
  TimePVA tpva2( time0,
                 PosVelAcc(pos0) );
  EXPECT_EQ( time0, tpva2.time );
  EXPECT_EQ( pos0,  tpva2.P.pos );
  EXPECT_EQ( 0.0,   tpva2.P.vel );
  EXPECT_EQ( 0.0,   tpva2.P.acc );
  //
  TimePVA tpva3( time0,
                 PosVelAcc(pos0, vel0) );
  EXPECT_EQ( time0, tpva3.time );
  EXPECT_EQ( pos0,  tpva3.P.pos );
  EXPECT_EQ( vel0,  tpva3.P.vel );
  EXPECT_EQ( 0.0,   tpva3.P.acc );
  //
  TimePVA tpva4( time0,
                 PosVelAcc(pos0, vel0, acc0) );
  EXPECT_EQ( time0, tpva4.time );
  EXPECT_EQ( pos0,  tpva4.P.pos );
  EXPECT_EQ( vel0,  tpva4.P.vel );
  EXPECT_EQ( acc0,  tpva4.P.acc );
}

TEST(TimePVATest, copy_constructor) {
  double time0 = 0.12;
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  //
  TimePVA src_tpva0( time0,
                     PosVelAcc(pos0, vel0, acc0) );
  TimePVA dest_tpva0( src_tpva0 );
  //
  EXPECT_EQ( src_tpva0.time, dest_tpva0.time );
  EXPECT_EQ( src_tpva0.P.pos,  dest_tpva0.P.pos );
  EXPECT_EQ( src_tpva0.P.vel,  dest_tpva0.P.vel );
  EXPECT_EQ( src_tpva0.P.acc,  dest_tpva0.P.acc );
  //
  double time1 = 0.98;
  double pos1  = 3.21;
  double vel1  = 6.54;
  double acc1  = 9.87;
  //
  src_tpva0.time = time1;
  src_tpva0.P.pos  = pos1;
  src_tpva0.P.vel  = vel1;
  src_tpva0.P.acc  = acc1;
  //
  EXPECT_EQ( time0, dest_tpva0.time );
  EXPECT_EQ( pos0,  dest_tpva0.P.pos );
  EXPECT_EQ( vel0,  dest_tpva0.P.vel );
  EXPECT_EQ( acc0,  dest_tpva0.P.acc );
  //
  PosVelAcc pva1( pos1, vel1, acc1 );
  TimePVA dest_tpva1( time1, pva1 );
  //
  EXPECT_EQ( time1, dest_tpva1.time );
  EXPECT_EQ( pos1,  dest_tpva1.P.pos );
  EXPECT_EQ( vel1,  dest_tpva1.P.vel );
  EXPECT_EQ( acc1,  dest_tpva1.P.acc );
}


TEST(TimePVATest, insert_operator ) {
  double time0 = 0.12;
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  //
  TimePVA src_tpva0( time0,
                     PosVelAcc(pos0, vel0, acc0) );
  // copy operator
  TimePVA dest_tpva0 = src_tpva0;
  //
  // check whether copy is succeeded.
  EXPECT_EQ(src_tpva0.time, dest_tpva0.time);
  EXPECT_EQ(src_tpva0.P.pos,  dest_tpva0.P.pos);
  EXPECT_EQ(src_tpva0.P.vel,  dest_tpva0.P.vel);
  EXPECT_EQ(src_tpva0.P.acc,  dest_tpva0.P.acc);
  //
  // check indipendent allocation
  double time1 = 0.98;
  double pos1 = 3.21;
  double vel1 = 6.54;
  double acc1 = 9.87;
  src_tpva0.time = time1;
  src_tpva0.P.pos  = pos1;
  src_tpva0.P.vel  = vel1;
  src_tpva0.P.acc  = acc1;
  EXPECT_NE(src_tpva0.P.pos, dest_tpva0.P.pos);
  EXPECT_NE(src_tpva0.P.vel, dest_tpva0.P.vel);
  EXPECT_NE(src_tpva0.P.acc, dest_tpva0.P.acc);
  // check copied dest is not changed if src is changed.
  EXPECT_EQ(pos0, dest_tpva0.P.pos);
  EXPECT_EQ(vel0, dest_tpva0.P.vel);
  EXPECT_EQ(acc0, dest_tpva0.P.acc);
  //
  // self insert
  dest_tpva0 = dest_tpva0;
  EXPECT_EQ(pos0, dest_tpva0.P.pos);
  EXPECT_EQ(vel0, dest_tpva0.P.vel);
  EXPECT_EQ(acc0, dest_tpva0.P.acc);
  // continuous insert
  TimePVA dest_tpva1;
  {
    TimePVA src_tpva2( time1,
                       PosVelAcc(pos1, vel1, acc1) );
    TimePVA dest_tpva2 = dest_tpva1 = src_tpva2 = src_tpva2;
    EXPECT_EQ(pos1, src_tpva2.P.pos);
    EXPECT_EQ(vel1, src_tpva2.P.vel);
    EXPECT_EQ(acc1, src_tpva2.P.acc);
    EXPECT_EQ(pos1, dest_tpva1.P.pos);
    EXPECT_EQ(vel1, dest_tpva1.P.vel);
    EXPECT_EQ(acc1, dest_tpva1.P.acc);
    EXPECT_EQ(pos1, dest_tpva2.P.pos);
    EXPECT_EQ(vel1, dest_tpva2.P.vel);
    EXPECT_EQ(acc1, dest_tpva2.P.acc);
  }
  EXPECT_EQ(pos1, dest_tpva1.P.pos);
  EXPECT_EQ(vel1, dest_tpva1.P.vel);
  EXPECT_EQ(acc1, dest_tpva1.P.acc);
}

TEST(TimePVATest, insert_operator2 ) {
  double time0 = 0.12;
  double pos0 = 1.23;
  double vel0 = 4.56;
  double acc0 = 7.89;
  //
  TimeVal<PosVelAcc> src_timeVal0( time0,
                                   PosVelAcc(pos0, vel0, acc0) );

  // copy operator
  TimePVA dest_tpva0 = src_timeVal0;
  // check whether copy is succeeded.
  EXPECT_EQ(src_timeVal0.time,       dest_tpva0.time);
  EXPECT_EQ(src_timeVal0.value.pos,  dest_tpva0.value.pos);
  EXPECT_EQ(src_timeVal0.value.vel,  dest_tpva0.value.vel);
  EXPECT_EQ(src_timeVal0.value.acc,  dest_tpva0.value.acc);
  EXPECT_EQ(src_timeVal0.value.pos,  dest_tpva0.P.pos);
  EXPECT_EQ(src_timeVal0.value.vel,  dest_tpva0.P.vel);
  EXPECT_EQ(src_timeVal0.value.acc,  dest_tpva0.P.acc);
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TPQueueTest, push_get_intervaltime_size_clear) {
  TimeVal<double> tp0(0.0, 0.0);
  TimeVal<double> tp1(1.0, 10.001);
  TimeVal<double> tp2(2.0, 20.002);
  TimeVal<double> tp3(3.0, 30.003);

  TPQueue tp_queue;
  // push TimeVal<double>
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue.push(tp1), SPLINE_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue.push(tp2), SPLINE_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue.push(tp3), SPLINE_SUCCESS ); // index=3
  // get()
  EXPECT_EQ( tp_queue.get(0).time, tp0.time );
  EXPECT_EQ( tp_queue.get(1).time, tp1.time );
  EXPECT_EQ( tp_queue.get(2).time, tp2.time );
  EXPECT_EQ( tp_queue.get(3).time, tp3.time );
  EXPECT_EQ( tp_queue.get(0).value, tp0.value );
  EXPECT_EQ( tp_queue.get(1).value, tp1.value );
  EXPECT_EQ( tp_queue.get(2).value, tp2.value );
  EXPECT_EQ( tp_queue.get(3).value, tp3.value );
  // dT()
  EXPECT_EQ( tp_queue.dT(0), tp1.time - tp0.time );
  EXPECT_EQ( tp_queue.dT(1), tp2.time - tp1.time );
  EXPECT_EQ( tp_queue.dT(2), tp3.time - tp2.time );
  EXPECT_THROW( tp_queue.dT(3), InvalidIndexAccess );
  // size()
  EXPECT_EQ( tp_queue.size(), 4 );
  // clear()
  tp_queue.clear();
  EXPECT_EQ( tp_queue.size(), 0 );
  //
  TPQueue tp_queue2;
  // push time, position directly
  EXPECT_EQ( tp_queue2.push_on_clocktime(0.0, 0.0),    SPLINE_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue2.push_on_clocktime(1.0, 10.001), SPLINE_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue2.push_on_clocktime(2.0, 20.002), SPLINE_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue2.push_on_clocktime(3.0, 30.003), SPLINE_SUCCESS ); // index=3
  //
  EXPECT_EQ( tp_queue2.get(0).time, tp0.time );
  EXPECT_EQ( tp_queue2.get(1).time, tp1.time );
  EXPECT_EQ( tp_queue2.get(2).time, tp2.time );
  EXPECT_EQ( tp_queue2.get(3).time, tp3.time );
  EXPECT_EQ( tp_queue2.get(0).value, tp0.value );
  EXPECT_EQ( tp_queue2.get(1).value, tp1.value );
  EXPECT_EQ( tp_queue2.get(2).value, tp2.value );
  EXPECT_EQ( tp_queue2.get(3).value, tp3.value );
  //
  EXPECT_EQ( tp_queue2.size(), 4 );
  tp_queue2.clear();
  EXPECT_EQ( tp_queue2.size(), 0 );
}

TEST(TPQueueTest, insert_operator){
  TimeVal<double> tp0(0.0, 0.0);
  TimeVal<double> tp1(1.0, 10.001);
  TimeVal<double> tp2(2.0, 20.002);
  TimeVal<double> tp3(3.0, 30.003);

  TPQueue tp_queue_src;
  TPQueue tp_queue_dest;
  //
  EXPECT_THROW( (tp_queue_dest = tp_queue_src), InvalidIndexAccess);
  //
  EXPECT_EQ( tp_queue_src.push(tp0), SPLINE_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue_src.push(tp1), SPLINE_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue_src.push(tp2), SPLINE_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue_src.push(tp3), SPLINE_SUCCESS ); // index=3
  //
  tp_queue_dest = tp_queue_src;
  //
  EXPECT_EQ( tp_queue_dest.get(0).time, tp_queue_src.get(0).time );
  EXPECT_EQ( tp_queue_dest.get(1).time, tp_queue_src.get(1).time );
  EXPECT_EQ( tp_queue_dest.get(2).time, tp_queue_src.get(2).time );
  EXPECT_EQ( tp_queue_dest.get(3).time, tp_queue_src.get(3).time );
  EXPECT_EQ( tp_queue_dest.get(0).value, tp_queue_src.get(0).value );
  EXPECT_EQ( tp_queue_dest.get(1).value, tp_queue_src.get(1).value );
  EXPECT_EQ( tp_queue_dest.get(2).value, tp_queue_src.get(2).value );
  EXPECT_EQ( tp_queue_dest.get(3).value, tp_queue_src.get(3).value );
}

TEST(TPQueueTest, push_set){
  TimeVal<double> tp0(0.0, 0.0);
  TimeVal<double> tp0_swap(0.0, -77.7);
  TimeVal<double> tp0_swap_NG(1.0, -77.7);
  TimeVal<double> tp1(1.0, 10.001);
  TimeVal<double> tp1_swap(1.0000001, 15.015);
  TimeVal<double> tp1_swap_NG(0.0, 15.015);
  TimeVal<double> tp2(2.0, 20.002);
  TimeVal<double> tp3(3.0, 30.003);
  TimeVal<double> tp3_swap(3.3, -30.003);

  TPQueue tp_queue;
  // push
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), SPLINE_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), SPLINE_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp3), SPLINE_SUCCESS ); // index=3
  EXPECT_EQ( tp_queue.push(tp0), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp3), SPLINE_INVALID_INPUT_TIME );
  // set
  EXPECT_EQ( tp_queue.set(0, tp0_swap), SPLINE_SUCCESS );
  EXPECT_EQ( tp_queue.set(0, tp0_swap_NG), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.set(1, tp1_swap), SPLINE_SUCCESS );
  EXPECT_EQ( tp_queue.set(1, tp1_swap_NG), SPLINE_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.set(6, tp3_swap), SPLINE_INVALID_INPUT_INDEX );
}


TEST(TPQueueTest, dump) {
  TimeVal<double> tp0( 0.0,    0.0   );
  TimeVal<double> tp1( 1.025, 10.001 );
  TimeVal<double> tp2( 2.037, 20.002 );
  TimeVal<double> tp3( 3.048, 30.003 );

  TPQueue tp_queue0;
  tp_queue0.push( tp0 ); // index=0
  tp_queue0.push( tp1 ); // index=1
  tp_queue0.push( tp2 ); // index=2
  tp_queue0.push( tp3 ); // index=3

  std::string buf;
  tp_queue0.dump( buf );
  std::cout << buf;
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TPVAQueueTest, dump) {
  TimePVA tpva0( 0.0,   PosVelAcc(  5.005,  -5.005, 0.0   ) );
  TimePVA tpva1( 1.025, PosVelAcc( 10.001, -10.001, 0.001 ) );
  TimePVA tpva2( 2.037, PosVelAcc( 20.002, -20.002, 0.002 ) );
  TimePVA tpva3( 3.048, PosVelAcc( 30.003, -30.003, 0.003 ) );

  TPVAQueue tpva_queue0;
  tpva_queue0.push( tpva0 ); // index=0
  tpva_queue0.push( tpva1 ); // index=1
  tpva_queue0.push( tpva2 ); // index=2
  tpva_queue0.push( tpva3 ); // index=3

  std::string buf;
  tpva_queue0.dump( buf );
  std::cout << buf;
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TPVAListQueueTest, dump) {
  PosVelAcc pva0_0(  5.005,  -5.005,  0.0   );
  PosVelAcc pva0_1(  0.211,  11.212, -3.222 );
  PosVelAcc pva0_2( -3.043, -98.555, 11.444 );
  PVAList pva_list0;
  pva_list0.push_back( pva0_0 );
  pva_list0.push_back( pva0_1 );
  pva_list0.push_back( pva0_2 );
  //
  PosVelAcc pva1_0(  6.015,  -6.015,  1.010 );
  PosVelAcc pva1_1(  1.221,  12.222, -4.232 );
  PosVelAcc pva1_2( -2.032, -97.544, 10.433 );
  PVAList pva_list1;
  pva_list1.push_back( pva1_0 );
  pva_list1.push_back( pva1_1 );
  pva_list1.push_back( pva1_2 );
  //
  PosVelAcc pva2_0(  7.025,  -7.025,  2.020 );
  PosVelAcc pva2_1(  2.231,  13.232, -5.242 );
  PosVelAcc pva2_2( -1.022, -96.533,  9.422 );
  PVAList pva_list2;
  pva_list2.push_back( pva2_0 );
  pva_list2.push_back( pva2_1 );
  pva_list2.push_back( pva2_2 );
  //
  PosVelAcc pva3_0(  8.035,  -8.035,  3.030 );
  PosVelAcc pva3_1(  3.241,  14.242, -6.252 );
  PosVelAcc pva3_2( -0.011, -95.522,  8.411 );
  PVAList pva_list3;
  pva_list3.push_back( pva3_0 );
  pva_list3.push_back( pva3_1 );
  pva_list3.push_back( pva3_2 );
  //
  TimePVAList tpva_list0( 0.0,   pva_list0 );
  TimePVAList tpva_list1( 1.025, pva_list1 );
  TimePVAList tpva_list2( 2.037, pva_list2 );
  TimePVAList tpva_list3( 3.048, pva_list3 );

  TPVAListQueue tpva_list_queue0;
  tpva_list_queue0.push( tpva_list0 ); // index=0
  tpva_list_queue0.push( tpva_list1 ); // index=1
  tpva_list_queue0.push( tpva_list2 ); // index=2
  tpva_list_queue0.push( tpva_list3 ); // index=3

  std::string buf;
  tpva_list_queue0.dump( buf );
  std::cout << buf;
}
