#ifndef CPR_LOGGER_H
#define CPR_LOGGER_H

#include <iostream>

namespace drobot
{

  class Logger
  {
  private:
    bool enabled;
    int level;

    std::ostream *stream;

    std::ofstream *nullStream; //i.e /dev/null

  public:
    enum logLevels
    {
      ERROR_LEV,
      EXCEPTION,
      WARNING,
      INFO,
      DETAIL
    };
    static const char *levelNames[]; // strings indexed by enumeration. 

  private:
    Logger();

    ~Logger();

    void close();

  public:
    static Logger &instance();

    std::ostream &entry(enum logLevels level, const char *file = 0, int line = -1);

    void setEnabled(bool enabled);

    void setLevel(enum logLevels newLevel);

    void setStream(std::ostream *stream);

    void hookFatalSignals();

    friend void loggerTermHandler(int signal);
  };

  void loggerTermHandler(int signal);

}; // namespace drobot

// convenience macros
#define CPR_LOG(level) (drobot::Logger::instance().entry((level), __FILE__, __LINE__ ))
#define CPR_ERR()      CPR_LOG(drobot::Logger::ERROR)
#define CPR_EXCEPT()   (drobot::Logger::instance().entry(drobot::Logger::EXCEPTION))
#define CPR_WARN()     CPR_LOG(drobot::Logger::WARNING)
#define CPR_INFO()     CPR_LOG(drobot::Logger::INFO)
#define CPR_DTL()      CPR_LOG(drobot::Logger::DETAIL)

#endif //CPR_LOGGER_H