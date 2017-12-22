
#ifndef SIMPLE_EXCEPT_H
#define SIMPLE_EXCEPT_H

#include <string>
#include <stdexcept>
#include <cstdio>
#include <stdarg.h>


#if(1)
#define FORMATTED_ERROR_CONS  \
    va_list ap; \
    va_start(ap, format); \
    char * fstr; \
    vasprintf(&fstr, format.c_str(), ap); \
    if(fstr != NULL) { \
        msg = fstr; \
        free(fstr); \
    } \
    va_end(ap);
#else
#define FORMATTED_ERROR_CONS \
    va_list ap; \
    va_start(ap, format); \
    char fstr[1024]; \
    vsnprintf(fstr, 1024, format.c_str(), ap); \
    msg = fstr; \
    va_end(ap);
#endif

class FormattedError: public std::exception {
  protected:
    std::string msg;
  public:
    FormattedError(const std::string & format, ...) {FORMATTED_ERROR_CONS}
    virtual ~FormattedError() throw() {}
    virtual const char* what() const throw() {return msg.c_str();}
};

#define FORMATTED_ERROR_CLASS(className, superClassName) \
class className: public superClassName { \
  protected: \
    std::string msg; \
  public: \
    className(const std::string & format, ...) {FORMATTED_ERROR_CONS} \
    virtual ~className() throw() {} \
    virtual const char* what() const throw() {return msg.c_str();} \
};

#endif // SIMPLE_EXCEPT_H
