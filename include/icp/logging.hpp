#ifndef ICP_LOGGING_HPP
#define ICP_LOGGING_HPP

class NullStream {
    public:
    NullStream() { }
    template<typename T> NullStream& operator<<(T const&) { return *this; }
};

#ifdef GLOG_ENABLED 
  #define GLOG_ENABLED 1
  #include <glog/logging.h>
#else
  #define GLOG_ENABLED 0
  #define WARNING
  #define INFO
  #define FATAL
  #define LOG(type) NullStream() 
  #define DLOG(type) NullStream() 
#endif

#endif
