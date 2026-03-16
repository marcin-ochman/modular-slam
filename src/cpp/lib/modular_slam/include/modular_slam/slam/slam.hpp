#ifndef MODULAR_SLAM_SLAM_HPP_
#define MODULAR_SLAM_SLAM_HPP_

#include "modular_slam/core/pose.hpp"
#include "modular_slam/frontend/visual_frontend.hpp"

#include <optional>

namespace modular_slam
{

template <typename Propagator, typename Backend, typename Map, typename... Frontends>
class Slam
{
  public:
    Slam(Propagator propagator, Backend backend, Map map, Frontends... frontends)
        : mPropagator(propagator), mBackend(backend), mMap(map), mFrontends(frontends...)
    {
    }

    template <typename SensorData>
    void feed([[maybe_unused]] const SensorData& data) noexcept
    {
        static constexpr auto frontendIndex = frontendIndexFor<SensorData>();

        static_assert(frontendIndex.has_value(), "No frontend supporting input data found");

        auto& frontend = std::get<frontendIndex.value()>(mFrontends);

        const auto prediction = mPropagator.predict(data.acquisition.timestamp);
        const auto result = frontend.process(data, prediction);

        // if(result.has_value())
        // {
        //     mBackend.addFactor(*result);

        //     flushDependentSensors(data.timestamp);

        //     if(mBackend.shouldOptimize())
        //     {
        //         mBackend.optimize();
        //         mProp.correct(mBackend.get_state());
        //     }
        // }
    }

    Pose currentPose() { return Pose{}; }
    const Map& map() const { return mMap; }

  private:
    Propagator mPropagator;
    Backend mBackend;
    Map mMap;
    std::tuple<Frontends...> mFrontends;

    template <typename T>
    static constexpr std::optional<std::size_t> frontendIndexFor()
    {
        std::size_t index = 0;
        std::optional<std::size_t> foundIndex;

        ((std::is_same_v<T, typename Frontends::InputData> ? (foundIndex = index, false) : (++index, true)) && ...);

        return foundIndex;
    }
};

} // namespace modular_slam

#endif // MODULAR_SLAM_SLAM_HPP_
