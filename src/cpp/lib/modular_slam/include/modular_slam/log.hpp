#ifndef MODULAR_SLAM_LOG_HPP
#define MODULAR_SLAM_LOG_HPP

#include <spdlog/spdlog.h>
#include <string_view>

#ifdef NDEBUG
#define MS_LOG_DEBUG(...) (void)0
#define MS_LOG_TRACE(...) (void)0
#else
#define MS_LOG_DEBUG(...) ::mslam::log::debug(__VA_ARGS__)
#define MS_LOG_TRACE(...) ::mslam::log::trace(__VA_ARGS__)
#endif

#define MS_LOG_INFO(...) ::mslam::log::info(__VA_ARGS__)
#define MS_LOG_ERROR(...) ::mslam::log::error(__VA_ARGS__)

namespace mslam::log
{

namespace impl
{
std::shared_ptr<spdlog::logger> getLogger();
}

template <typename... Args>
inline void info(spdlog::format_string_t<Args...> fmt, Args&&... args)
{
    impl::getLogger()->info(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void debug(spdlog::format_string_t<Args...> fmt, Args&&... args)
{
    impl::getLogger()->debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(spdlog::format_string_t<Args...> fmt, Args&&... args)
{
    impl::getLogger()->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(spdlog::format_string_t<Args...> fmt, Args&&... args)
{
    impl::getLogger()->error(fmt, std::forward<Args>(args)...);
}

} // namespace mslam::log
#endif
