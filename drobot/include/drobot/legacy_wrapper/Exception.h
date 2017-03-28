#ifndef CPR_EXCEPTION_H
#define CPR_EXCEPTION_H

#include <iostream>

namespace drobot
{

  class Exception
  {
  public:
    const char *message;

  protected:
    Exception(const char *msg = "none") : message(msg)
    {
    }
  };

}; // namespace drobot

#endif // CPR_EXCEPTION_H