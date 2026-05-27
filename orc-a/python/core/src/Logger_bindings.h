#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "orc/util/Logger.h"

namespace nb = nanobind;
using namespace nb::literals;

void add_Logger(nb::module_& log_submodule) {
    nb::enum_<orc::log::Level>(log_submodule, "LogLevel")
        .value("Fatal", orc::log::Level::Fatal)
        .value("Error", orc::log::Level::Error)
        .value("Warning", orc::log::Level::Warning)
        .value("Info", orc::log::Level::Info)
        .value("Debug", orc::log::Level::Debug)
        .export_values();  // This makes the enum values accessible in the Python module

    log_submodule.def("start_logging", &orc::log::start_logging, "level"_a = orc::log::Level::Info);
    log_submodule.def("write", &orc::log::write, "level"_a, "msg"_a);
    log_submodule.def("write_fatal", &orc::log::write_fatal, "msg"_a);
    log_submodule.def("write_error", &orc::log::write_error, "msg"_a);
    log_submodule.def("write_warning", &orc::log::write_warning, "msg"_a);
    log_submodule.def("write_info", &orc::log::write_info, "msg"_a);
    log_submodule.def("write_debug", &orc::log::write_debug, "msg"_a);
    log_submodule.def("write_double", &orc::log::write_double, "msg"_a);
    log_submodule.def("write_int", &orc::log::write_int, "number"_a);
}
