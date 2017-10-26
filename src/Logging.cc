#include "Logging.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/attributes/function.hpp>
#include <boost/log/expressions/formatters/date_time.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/formatter_parser.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/empty_deleter.hpp>
#include <fstream>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace attrs = boost::log::attributes;

namespace ORB_SLAM2 {
int SystemLogger::totalFrameCount = 0;
int SystemLogger::currentFrameNum = 0;
const char* SystemLogger::LogLevelNames[SystemLogger::kLevelCount] =
{
  "STATUS",
  "INFO",
  "ERROR",
  "CRITIAL",
  "ERROR"
};


BOOST_LOG_ATTRIBUTE_KEYWORD(level, "Severity", SystemLogger::LogLevel)
BOOST_LOG_ATTRIBUTE_KEYWORD(scope, "Scope", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(frameNum, "FrameNum", int)
BOOST_LOG_ATTRIBUTE_KEYWORD(totalFrames, "TotalFrameCount", int)

BOOST_LOG_GLOBAL_LOGGER_INIT(sys_lg, src::severity_logger_mt<LogLevel>) {
    src::severity_logger_mt<SystemLogger::LogLevel> sys_lg;

    // add attributes
    logging::core::get()->add_global_attribute("Scope", attrs::named_scope());
    sys_lg.add_attribute("FrameNum", attrs::make_function([&]{return SystemLogger::currentFrameNum;}));
    sys_lg.add_attribute("TotalFrameCount", attrs::make_function([&]{return SystemLogger::totalFrameCount;}));

    // add mapping for log level names
    boost::log::register_simple_formatter_factory< SystemLogger::LogLevel, char >("Severity");

       // Construct the sinks
    typedef sinks::synchronous_sink< sinks::text_ostream_backend > text_sink;
    boost::shared_ptr< text_sink > console_sink = boost::make_shared< text_sink >();
    // boost::shared_ptr< text_sink > status_sink = boost::make_shared< text_sink >();

    // Add a stream to write log to
    // status_sink->locked_backend()->add_stream(boost::make_shared< std::ofstream >(mkStatusLogFile));

    //Tell it to write directly instead of when the object goes out of scope
    // status_sink->locked_backend()->auto_flush(true);

    // We have to provide an empty deleter to avoid destroying the global stream object
    boost::shared_ptr< std::ostream > stream(&std::clog, logging::empty_deleter());
    console_sink->locked_backend()->add_stream(stream);

    // Register the sinks in the logging core
    logging::core::get()->add_sink(console_sink);
    // logging::core::get()->add_sink(status_sink);

    // specify the format of the log messages
    logging::formatter formatter = expr::stream
        << "[" << level << "]"
        << "<" << expr::format_named_scope("Scope", boost::log::keywords::format = "%n") << ">"
        << " - " << expr::smessage;
    // logging::formatter status_formatter = expr::stream
    //     << "<" << expr::format_named_scope("Scope", boost::log::keywords::format = "%n") << "> "
    //     << frameNum << "/" << totalFrames
    //     << " - " << expr::smessage;

    console_sink->set_formatter(formatter);
    // status_sink->set_formatter(status_formatter);

    // don't log status messages to console, instead log them to file
    console_sink->set_filter(level != SystemLogger::LogLevel::STATUS);
    // status_sink->set_filter(level == SystemLogger::LogLevel::STATUS);

    return sys_lg;
}

}
