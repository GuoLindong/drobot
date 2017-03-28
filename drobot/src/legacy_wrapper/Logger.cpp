#include <iostream>
#include <fstream>
#include <signal.h>
#include <unistd.h>

using namespace std;

#include "drobot/legacy_wrapper/Logger.h"

namespace drobot
{

  const char *Logger::levelNames[] = {"ERROR", "EXCEPTION", "WARNING", "INFO", "DETAIL"};

  void loggerTermHandler(int signum)
  {
    Logger::instance().close();

    if ((signum == SIGABRT) || (signum == SIGSEGV))
    {
      /* We need to catch these so we can flush out any last messages.
       * having done this, probably the most consistent thing to do
       * is re-raise the signal.  (We certainly don't want to just
       * ignore these) */
      signal(signum, SIG_DFL);
      kill(getpid(), signum);
    }
  }

  Logger &Logger::instance()
  {
    static Logger instance;
    return instance;
  }

  Logger::Logger() :
      enabled(true),
      level(WARNING),
      stream(&cerr)
  {
    nullStream = new ofstream("/dev/null");
  }

  Logger::~Logger()
  {
    close();
  }

  void Logger::close()
  {
    // The actual output stream is owned by somebody else, we only need to flush it
    stream->flush();

    nullStream->close();
    delete nullStream;
    nullStream = 0;
  }

  std::ostream &Logger::entry(enum logLevels msg_level, const char *file, int line)
  {
    if (!enabled) { return *nullStream; }
    if (msg_level > this->level) { return *nullStream; }

    /* Construct the log entry tag */
    // Always the level of the message
    *stream << levelNames[msg_level];
    // If file/line information is provided, need to print it with brackets:
    if (file || (line >= 0))
    {
      *stream << " (";
      if (file) { *stream << file; }
      // Only want a comma if we have both items
      if (file && (line >= 0)) { *stream << ","; }
      if (line >= 0) { *stream << line; }
      *stream << ")";
    }
    *stream << ": ";
    return *stream;
  }

  void Logger::setEnabled(bool en)
  {
    enabled = en;
  }

  void Logger::setLevel(enum logLevels newLevel)
  {
    level = newLevel;
  }

  void Logger::setStream(ostream *newStream)
  {
    stream->flush();
    stream = newStream;
  }

  void Logger::hookFatalSignals()
  {
    signal(SIGINT, loggerTermHandler);
    signal(SIGTERM, loggerTermHandler);
    /* If there's an abort or segfault in Logger.close(), well...
     * we're pretty much totally screwed anyway. */
    signal(SIGABRT, loggerTermHandler);
    signal(SIGSEGV, loggerTermHandler);
  }

}; // namespace drobot

