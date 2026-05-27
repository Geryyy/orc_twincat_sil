#pragma once

#include <stdarg.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>

#ifdef TC_VER
// in twincat environment
#include "TcInterfaces.h"
#include "TcRtInterfaces.h"
#else
// full C++ support
#include <chrono>
#include <iostream>
#endif

namespace orc::log {

enum class Level { Fatal, Error, Warning, Info, Debug };

// Logging class.

class Logger {
public:
    Level max_level = Level::Info;

#if TC_VER
    CTcTrace* tc_trace = nullptr;
    Logger(Level max_logging_level, CTcTrace* trace) {
        max_level = max_logging_level;
        tc_trace = trace;
    }
#else
    Logger(Level max_logging_level) { max_level = max_logging_level; }
#endif
    // void info(const std::string& msg){
    //     std::string report = "Info: " + msg;
    // }
    ~Logger() {}

public:
    void log(Level s, const std::string& msg) {
        if (s <= max_level) {
            std::string str = "orc::log::";
#ifdef TC_VER
            if (tc_trace != nullptr) {
                switch (s) {
                    case Level::Fatal:
                        str += "Fatal: " + msg;
                        break;
                    case Level::Error:
                        str += "Error: " + msg;
                        break;
                    case Level::Warning:
                        str += "Warning: " + msg;
                        break;
                    case Level::Info:
                        str += "Info: " + msg;
                        break;
                    case Level::Debug:
                        str += "Debug: " + msg;
                        break;
                    default:
                        str += "Invalid Log Level: " + msg;
                }

                std::vector<char> cstr(str.c_str(), str.c_str() + str.size() + 1);
                tc_trace->Log(tlAlways, cstr.data());
            }
#else
            switch (s) {
                case Level::Fatal:
                    std::cout << str + "Fatal: " << msg << std::endl;
                    break;
                case Level::Error:
                    std::cout << str + "Error: " << msg << std::endl;
                    break;
                case Level::Warning:
                    std::cout << str + "Warning: " << msg << std::endl;
                    break;
                case Level::Info:
                    std::cout << str + "Info: " << msg << std::endl;
                    break;
                case Level::Debug:
                    std::cout << str + "Debug: " << msg << std::endl;
                    break;
                default:
                    std::cout << str + "Invalid logger level: " << msg << std::endl;
            }
#endif
        } else {
            // max. allowed log level is too low. print nothing.
        }
    }
};

// extern Logger* arc_default_logger;

#if TC_VER
void start_logging_twincat(TcTraceLevel trace_level_max, CTcTrace* trace);
#else
void start_logging(Level log_level);
#endif

/* default logging */
void write(Level s, const std::string& msg);

/* convenience functions */
void write_fatal(const std::string& msg);
void write_error(const std::string& msg);
void write_warning(const std::string& msg);
void write_info(const std::string& msg);
void write_debug(const std::string& msg);
void write_double(double number);
void write_int(int number);

/* timing functions */
#if TC_VER
void start_timing(ITcTask* spTask);
#else
void start_timing();
#endif

void tic(int64_t* pstart_time);
double toc(int64_t start_time);      // return time since tic() in microseconds
void toc_write(int64_t start_time);  // write time directly as debug output

}  // namespace orc::log
