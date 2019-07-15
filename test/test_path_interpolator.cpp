#include <gtest/gtest.h>
#include "path_interpolator.hpp"

using namespace interp;

TEST(PathExceptionTest, THROW) {
  try {
    THROW (PathException, "This is Message.");
  } catch(PathException& pe) {
    EXPECT_TRUE(  std::string(pe.what()).find("[PathException] This is Message.") );
    std::cout << pe.what() << std::endl;
  }
}

TEST(RetValTest, construct) {
  /// case1 double
  /// no argument constructor
  RetVal<int> retval_int;
  EXPECT_EQ( PATH_NOT_RETURN, retval_int.retcode );
  int int_val = 5;
  EXPECT_NE( int_val, retval_int.value );

  retval_int.retcode = PATH_SUCCESS;
  EXPECT_EQ( PATH_SUCCESS, retval_int.retcode );
  retval_int.value = int_val;
  EXPECT_EQ( int_val, retval_int.value );

  /// argument constructor
  RetVal<int> retval_int2(PATH_SUCCESS, int_val);
  EXPECT_EQ( PATH_SUCCESS, retval_int2.retcode );
  EXPECT_EQ( int_val, retval_int2.value );

  /// case2 double
  RetVal<double> retval_double;
  EXPECT_EQ( PATH_NOT_RETURN, retval_double.retcode );
  double double_val = 12.34;
  EXPECT_NE( double_val, retval_double.value );
  retval_double.value = double_val;
  EXPECT_EQ( double_val, retval_double.value );

  /// case3 std::string
  RetVal<std::string> retval_string;
  EXPECT_EQ( PATH_NOT_RETURN, retval_string.retcode );
  EXPECT_EQ( "", retval_string.value );
  std::string string_val = "TEST";
  EXPECT_NE( string_val, retval_string.value );
  retval_string.value = string_val;
  EXPECT_EQ( string_val, retval_string.value );
}

TEST(RetValTest, copy_operator) {
  /// copy operator
  int int_val = 5;
  RetVal<int> retval_int;
  retval_int = RetVal<int>(PATH_SUCCESS, int_val);
  EXPECT_EQ( PATH_SUCCESS, retval_int.retcode );
  EXPECT_EQ( int_val, retval_int.value );
}

/////////////////////////////////////////////////////////////////////////////////////////

TEST(TPQueueTest, push_get_dT_size_clear){
  TimePosition tp0(0.0, 0.0);
  TimePosition tp1(1.0, 10.001);
  TimePosition tp2(2.0, 20.002);
  TimePosition tp3(3.0, 30.003);

  TPQueue tp_queue;
  // push TimePosition
  EXPECT_EQ( tp_queue.push(tp0), PATH_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue.push(tp1), PATH_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue.push(tp2), PATH_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue.push(tp3), PATH_SUCCESS ); // index=3
  // get()
  EXPECT_EQ( tp_queue.get(0).time, tp0.time );
  EXPECT_EQ( tp_queue.get(1).time, tp1.time );
  EXPECT_EQ( tp_queue.get(2).time, tp2.time );
  EXPECT_EQ( tp_queue.get(3).time, tp3.time );
  EXPECT_EQ( tp_queue.get(0).position, tp0.position );
  EXPECT_EQ( tp_queue.get(1).position, tp1.position );
  EXPECT_EQ( tp_queue.get(2).position, tp2.position );
  EXPECT_EQ( tp_queue.get(3).position, tp3.position );
  // dT()
  RetVal<double> retval = tp_queue.dT(0);
  EXPECT_EQ( retval.retcode, PATH_SUCCESS );
  EXPECT_EQ( retval.value, tp1.time - tp0.time );
  retval = tp_queue.dT(1);
  EXPECT_EQ( retval.retcode, PATH_SUCCESS );
  EXPECT_EQ( retval.value, tp2.time - tp1.time );
  retval = tp_queue.dT(2);
  EXPECT_EQ( retval.retcode, PATH_SUCCESS );
  EXPECT_EQ( retval.value, tp3.time - tp2.time );
  retval = tp_queue.dT(3);
  EXPECT_EQ( retval.retcode, PATH_INVALID_INPUT_INDEX );
  // size()
  EXPECT_EQ( tp_queue.size(), 4 );
  // clear()
  tp_queue.clear();
  EXPECT_EQ( tp_queue.size(), 0 );
  //
  TPQueue tp_queue2;
  // push time, position directly
  EXPECT_EQ( tp_queue2.push(0.0, 0.0),    PATH_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue2.push(1.0, 10.001), PATH_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue2.push(2.0, 20.002), PATH_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue2.push(3.0, 30.003), PATH_SUCCESS ); // index=3
  //
  EXPECT_EQ( tp_queue2.get(0).time, tp0.time );
  EXPECT_EQ( tp_queue2.get(1).time, tp1.time );
  EXPECT_EQ( tp_queue2.get(2).time, tp2.time );
  EXPECT_EQ( tp_queue2.get(3).time, tp3.time );
  EXPECT_EQ( tp_queue2.get(0).position, tp0.position );
  EXPECT_EQ( tp_queue2.get(1).position, tp1.position );
  EXPECT_EQ( tp_queue2.get(2).position, tp2.position );
  EXPECT_EQ( tp_queue2.get(3).position, tp3.position );
  //
  EXPECT_EQ( tp_queue2.size(), 4 );
  tp_queue2.clear();
  EXPECT_EQ( tp_queue2.size(), 0 );
}

TEST(TPQueueTest, push_set){
  TimePosition tp0(0.0, 0.0);
  TimePosition tp0_swap(0.0, -77.7);
  TimePosition tp0_swap_NG(1.0, -77.7);
  TimePosition tp1(1.0, 10.001);
  TimePosition tp1_swap(1.0000001, 15.015);
  TimePosition tp1_swap_NG(0.0, 15.015);
  TimePosition tp2(2.0, 20.002);
  TimePosition tp3(3.0, 30.003);
  TimePosition tp3_swap(3.3, -30.003);

  TPQueue tp_queue;
  // push
  EXPECT_EQ( tp_queue.push(tp0), PATH_SUCCESS ); // index=0
  EXPECT_EQ( tp_queue.push(tp0), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), PATH_SUCCESS ); // index=1
  EXPECT_EQ( tp_queue.push(tp0), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), PATH_SUCCESS ); // index=2
  EXPECT_EQ( tp_queue.push(tp0), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp3), PATH_SUCCESS ); // index=3
  EXPECT_EQ( tp_queue.push(tp0), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp1), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp2), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.push(tp3), PATH_INVALID_INPUT_TIME );
  // set
  EXPECT_EQ( tp_queue.set(0, tp0_swap), PATH_SUCCESS );
  EXPECT_EQ( tp_queue.set(0, tp0_swap_NG), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.set(1, tp1_swap), PATH_SUCCESS );
  EXPECT_EQ( tp_queue.set(1, tp1_swap_NG), PATH_INVALID_INPUT_TIME );
  EXPECT_EQ( tp_queue.set(6, tp3_swap), PATH_INVALID_INPUT_INDEX );
}
