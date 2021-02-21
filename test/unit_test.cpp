#include <gtest/gtest.h>
#include "plog/Log.h"
#include <plog/Appenders/ColorConsoleAppender.h>

/// set logging level of plog
/// @param level logging level(fatal, error, info, debug, verbose, none)
void set_logging_level(const std::string& level) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;

  if (level == "fatal" || level == "FATAL") {
    plog::init(plog::fatal, &consoleAppender);
  } else if (level == "error" || level == "ERROR") {
    plog::init(plog::error, &consoleAppender);
  } else if (level == "info" || level == "INFO") {
    plog::init(plog::info, &consoleAppender);
  } else if (level == "debug" || level == "DEBUG") {
    plog::init(plog::debug, &consoleAppender);
  } else if (level == "verbose" || level == "VERBOSE") {
    plog::init(plog::verbose, &consoleAppender);
  } else if (level == "none" || level == "NONE") {
    plog::init(plog::none, &consoleAppender);
  } else {
    plog::init(plog::warning, &consoleAppender);
  }
}


int main(int argc, char* argv[])
{
  // (void)argc;
  // (void)argv;

  // google testの初期化
  ::testing::InitGoogleTest(&argc, argv);

  // configure logging level
  std::string level;
  if (const char* env_p = std::getenv("PLOG_LEVEL")) {
    level = std::string(env_p);
  }
  set_logging_level(level);

  // google testの実行
  return RUN_ALL_TESTS();
}
