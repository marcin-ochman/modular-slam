#include "modular_slam/log.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <print>
#include <spdlog/sinks/basic_file_sink.h>

namespace modular_slam::log::impl
{

class LogManager
{
  public:
    static LogManager& instance();
    std::shared_ptr<spdlog::logger> getLogger() { return mLogger; }

    ~LogManager() = default;
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;
    LogManager(LogManager&&) = delete;
    LogManager& operator=(LogManager&&) = delete;

  private:
    LogManager();
    std::shared_ptr<spdlog::logger> mLogger;
};

LogManager::LogManager()
{
    mLogger = spdlog::stdout_color_mt("modular_slam");
    mLogger->set_level(spdlog::level::trace);
    mLogger->set_error_handler([](const std::string& msg)
                               { std::print(stderr, "[MODULAR_SLAM LOG ERROR]: {}\n", msg); });
}

LogManager& LogManager::instance()
{
    static LogManager instance;

    return instance;
}

std::shared_ptr<spdlog::logger> getLogger()
{
    return LogManager::instance().getLogger();
}

} // namespace modular_slam::log::impl
