#pragma once

#include <stdlib.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include <vector>
namespace logger{
class logger
{
private:
   logger::logger(const std::string &  logger_name)
{
    std::string log_file_name = logger_name + "_log.txt";
    std::vector<spdlog::sink_ptr> sinks;

    auto sink_tocmd = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink_tocmd->set_level(spdlog::level::debug);
    sink_tocmd->set_pattern("[%^%l%$] %v");
    sinks.push_back(sink_tocmd);

    auto sink_tofile = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file_name, 1024 * 1024 * 10, 5, false);
    sink_tofile->set_level(spdlog::level::debug);
    
    spdlogger = std::make_shared<spdlog::logger>(logger_name, sinks.begin(), sinks.end());

    // spdlogger = 
    spdlogger->set_level(spdlog::level::debug);

    spdlog::register_logger(spdlogger);
}

public:
    logger(const logger &) = delete;
    logger &operator=(const logger &) = delete;
    static logger *GetInst( const std::string &logger_name)
    {

        static logger mysingle_logger(logger_name);
        return &mysingle_logger;
    }
    template <spdlog::level::level_enum Level = spdlog::level::info>
    void out(const std::string &msg)
    {
        spdlogger->log(Level, msg);
    }
    std::shared_ptr<spdlog::logger> spdlogger;
    ~logger() {};
};
    template <spdlog::level::level_enum Level = spdlog::level::info>
    void out(const std::string &msg)
    {
         logger*   my_logger=logger::GetInst("my_logger");
         my_logger->out<Level>(msg);
    }

}