#ifndef DEBUG_H
#define DEBUG_H

#define BOOST_LOG_DYN_LINK 1 //necessary for linking boost log dynamically

#include <boost/log/core/core.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <iostream>
#include <vector>


namespace src = boost::log::sources;

namespace ORB_SLAM2 {
class SystemLogger {
public:
    // current and total frame count
    static int totalFrameCount;
    static int currentFrameNum;

    // The log levels that can be used with SystemLogger
    static const int kLevelCount = 5;
    enum LogLevel
    {
        STATUS,
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };

    static const char* LogLevelNames[kLevelCount];
};

// names for the loglevels above
// needed for nameing the log levels
template< typename CharT, typename TraitsT >
std::basic_ostream< CharT, TraitsT >&
operator<< ( std::basic_ostream< CharT, TraitsT >& strm, SystemLogger::LogLevel lvl)
{
    const char* str = SystemLogger::LogLevelNames[lvl];
    if (lvl < SystemLogger::kLevelCount && lvl >= 0)
        strm << str;
    else
        strm << static_cast< int >(lvl);
    return strm;
}

// name of the output file
const std::string mkStatusLogFile = "orb_slam_status.log";

// create global logger
BOOST_LOG_GLOBAL_LOGGER(sys_lg, src::severity_logger_mt<SystemLogger::LogLevel>)
}

#define LOG(level)\
    BOOST_LOG_SEV(ORB_SLAM2::sys_lg::get(), ORB_SLAM2::SystemLogger::LogLevel::level)
#define LOG_SCOPE(name) BOOST_LOG_NAMED_SCOPE(name)

#ifdef DEBUG
    #define DLOG_IF(level, cond) if(cond) LOG(level)
    #define DLOG(level) LOG(level)
#else
    #define DLOG_IF(level, cond) if(false) LOG(level)
    #define DLOG(level) if(false) LOG(level)
#endif

#endif //DEBUG_H
