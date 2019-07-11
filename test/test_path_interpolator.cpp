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
