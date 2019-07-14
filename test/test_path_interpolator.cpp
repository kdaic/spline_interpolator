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
