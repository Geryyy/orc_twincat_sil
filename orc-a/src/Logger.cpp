#include "orc/util/Logger.h"

#ifdef TC_VER
// TwinCAT/Codesys RT C++ forbids <atomic>. The logger is initialized once at
// SAFEOP->OP transition before the cyclic task runs, so a plain pointer is safe.
static orc::log::Logger* orc_default_logger = nullptr;
#else
#include <atomic>
static std::atomic<orc::log::Logger*> orc_default_logger{nullptr};
#endif

#if TC_VER
void orc::log::start_logging_twincat(TcTraceLevel trace_level_max, CTcTrace* trace) {
    Level log_level;

    switch (trace_level_max) {
        case tlAlways:
            log_level = Level::Fatal;
            break;
        case tlError:
            log_level = Level::Error;
            break;
        case tlWarning:
            log_level = Level::Warning;
            break;
        case tlInfo:
            log_level = Level::Info;
            break;
        case tlVerbose:
            log_level = Level::Debug;
            break;
        default:
            log_level = Level::Fatal;
    }

    Logger* old = orc_default_logger;
    orc_default_logger = new Logger(log_level, trace);
    delete old;
}
#else
void orc::log::start_logging(Level log_level) {
    Logger* old = orc_default_logger.exchange(new Logger(log_level), std::memory_order_acq_rel);
    delete old;
}
#endif

/* default logging */
void orc::log::write(Level s, const std::string& msg) {
#ifdef TC_VER
    orc::log::Logger* lg = orc_default_logger;
#else
    orc::log::Logger* lg = orc_default_logger.load(std::memory_order_acquire);
#endif
    if (lg != nullptr) {
        lg->log(s, msg);
    }
}

void orc::log::write_fatal(const std::string& msg) {
    orc::log::write(orc::log::Level::Fatal, msg);
}

void orc::log::write_error(const std::string& msg) {
    orc::log::write(orc::log::Level::Error, msg);
}

void orc::log::write_warning(const std::string& msg) {
    orc::log::write(orc::log::Level::Warning, msg);
}

void orc::log::write_info(const std::string& msg) {
    orc::log::write(orc::log::Level::Info, msg);
}

void orc::log::write_debug(const std::string& msg) {
    orc::log::write(orc::log::Level::Debug, msg);
}

void orc::log::write_double(double number) {
    constexpr int str_len = 128;
    char output[str_len];
    snprintf(output, str_len, "%f", number);
    orc::log::write(orc::log::Level::Debug, output);
}

void orc::log::write_int(int number) {
    constexpr int str_len = 128;
    char output[str_len];
    snprintf(output, str_len, "%d", number);
    orc::log::write(orc::log::Level::Debug, output);
}

/* timing measurements */
// in microseconds

#if TC_VER
ITcTask* m_spTask = nullptr;

void orc::log::start_timing(ITcTask* spTask) {
    m_spTask = spTask;
}
#else
void orc::log::start_timing() {
    std::cout << "TODO: implement orc::log::start_timing" << std::endl;
}
#endif

void orc::log::tic(int64_t* pstart_time) {
#if TC_VER
    if (m_spTask != nullptr) {
        m_spTask->GetCurPentiumTime(pstart_time);
    } else {
        *pstart_time = 0;
    }
#else
    *pstart_time = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count() *
                   10;
#endif
}

double orc::log::toc(int64_t start_time) {
    int64_t end_time;
#if TC_VER
    if (m_spTask != nullptr) {
        m_spTask->GetCurPentiumTime(&end_time);
    } else {
        end_time = 0;
    }
#else
    end_time = std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now().time_since_epoch())
                   .count() *
               10;
#endif
    return static_cast<double>((end_time - start_time)) / 10.0;
}

void orc::log::toc_write(int64_t start_time) {
    constexpr int str_len = 128;
    char output[str_len];

    double duration = orc::log::toc(start_time);
    snprintf(output, str_len, "Execution Time: %f us.", duration);
    orc::log::write_debug(output);
}
