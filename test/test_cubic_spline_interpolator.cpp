#include <gtest/gtest.h>
#include "cubic_spline_interpolator.hpp"

namespace interp {

/// Friend Test Class of CubicSplineInterpolator
class CubicSplineTest: public ::testing::Test
{
protected:
  CubicSplineTest() {}

  CubicSplineInterpolator cubic_spline_;

  RetVal<std::vector<double> >
  m_tridiagonal_matrix_eq_solver( std::vector<double> d, const std::vector<double>& u,
                                const std::vector<double>& l, std::vector<double> p ) {
    return cubic_spline_.tridiagonal_matrix_eq_solver( d, u, l, p );
  }
};

}

using namespace interp;

TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_size_not_same ) {
  double d_array[] = {4, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1};
  std::vector<double> p(p_array, p_array + 1);

  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p ),
                InvalidArgumentSize );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_size ) {
  std::vector<double> d;
  std::vector<double> u;
  std::vector<double> l;
  std::vector<double> p;
  EXPECT_THROW( m_tridiagonal_matrix_eq_solver( d, u, l, p ),
                InvalidArgumentSize );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_invalid_argument_value_zero ) {
  double d_array[] = {0, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1, 0};
  std::vector<double> p(p_array, p_array + 2);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_INVALID_ARGUMENT_VALUE_ZERO );
}


TEST_F( CubicSplineTest, tri_matrix_eq_solver_list_size_is_1 ) {
  double d_array[] = {4};
  std::vector<double> d(d_array, d_array + 1);
  double u_array[] = {3};
  std::vector<double> u(u_array, u_array + 1);
  double l_array[] = {2};
  std::vector<double> l(l_array, l_array + 1);
  double p_array[] = {1};
  std::vector<double> p(p_array, p_array + 1);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], p[0] / d[0] ));
}

TEST_F( CubicSplineTest, tri_matrix_eq_solver_size_is_2 ) {
  double d_array[] = {4, 3};
  std::vector<double> d(d_array, d_array + 2);
  double u_array[] = {3, 2};
  std::vector<double> u(u_array, u_array + 2);
  double l_array[] = {2, 1};
  std::vector<double> l(l_array, l_array + 2);
  double p_array[] = {1, 0};
  std::vector<double> p(p_array, p_array + 2);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], 1.0/3.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[1], -1.0/9.0 ) );
}

TEST_F( CubicSplineTest, tri_matrix_eq_solver_size_is_3 ) {
  double d_array[] = {4, 3, 2};
  std::vector<double> d(d_array, d_array + 3);
  double u_array[] = {3, 2, 1};
  std::vector<double> u(u_array, u_array + 3);
  double l_array[] = {2, 1, -1};
  std::vector<double> l(l_array, l_array + 3);
  double p_array[] = {1, -1, -2};
  std::vector<double> p(p_array, p_array + 3);
  RetVal<std::vector<double> > ret = m_tridiagonal_matrix_eq_solver( d, u, l, p );
  EXPECT_EQ( ret.retcode, PATH_SUCCESS );
  EXPECT_TRUE( g_nearEq( ret.value[0], 1.0/13.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[1], 3.0/13.0 ) );
  EXPECT_TRUE( g_nearEq( ret.value[2], -23.0/26.0 ) );
}
