#include <gtest/gtest.h>


int main(int argc, char* argv[])
{
  // (void)argc;
  // (void)argv;

  // google testの初期化
  ::testing::InitGoogleTest(&argc, argv);

  // google testの実行
  return RUN_ALL_TESTS();
}
