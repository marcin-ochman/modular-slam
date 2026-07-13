#ifndef MODULAR_SLAM_SLAM_CONTEXT_HPP
#define MODULAR_SLAM_SLAM_CONTEXT_HPP

#include "modular_slam/core/channel.hpp"

#include <any>
#include <mutex>
#include <string>
#include <unordered_map>

namespace mslam
{

class SlamContext
{
  public:
    template <typename T>
    Channel<T>& getChannel(const std::string& name)
    {
        std::lock_guard lock{mRegistryMutex};

        const auto typeId = getTypeId<T>();
        auto& typeChannels = mRegistries[typeId];

        auto it = typeChannels.find(name);
        if(it == typeChannels.end())
        {
            it = typeChannels.emplace(name, std::make_any<Channel<T>>()).first;
        }

        return std::any_cast<Channel<T>&>(it->second);
    }

  private:
    template <typename T>
    static const void* getTypeId()
    {
        static const std::byte typeTag{0};

        return &typeTag;
    }

    std::unordered_map<void*, std::unordered_map<std::string, std::any>> mRegistries;
    std::mutex mRegistryMutex;
};
} // namespace mslam

#endif // MODULAR_SLAM_SLAM_CONTEXT_HPP
