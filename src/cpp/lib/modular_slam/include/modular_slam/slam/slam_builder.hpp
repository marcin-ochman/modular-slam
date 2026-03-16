#ifndef MODULAR_SLAM_FEATURE_SLAM_BUILDER_HPP
#define MODULAR_SLAM_FEATURE_SLAM_BUILDER_HPP

#include "modular_slam/slam/slam.hpp"

namespace modular_slam
{

struct NoPropagator
{
};

struct NoBackend
{
};

struct NoMap
{
};

template <typename Propagator = NoPropagator, typename Backend = NoBackend, typename Map = NoMap, typename... Frontends>
class SlamBuilder
{
    template <typename, typename, typename, typename...>
    friend class SlamBuilder;

  public:
    static auto create() { return SlamBuilder<NoPropagator, NoBackend, NoMap>{{}, {}, {}, std::tuple<>{}}; }

    template <typename NewProp, typename... Args>
    auto withPropagator(Args&&... args)
    {
        auto propagator = NewProp(std::forward<Args>(args)...);

        return SlamBuilder<NewProp, Backend, Map, Frontends...>(std::move(propagator), std::move(backend),
                                                                std::move(mMap), std::move(mFrontends));
    }

    template <typename NewBackend, typename... Args>
    auto withBackend(Args&&... args)
    {
        auto backend = NewBackend(std::forward<Args>(args)...);
        return SlamBuilder<Propagator, NewBackend, Map, Frontends...>(std::move(mPropagator), std::move(backend),
                                                                      std::move(mMap), std::move(mFrontends));
    }

    template <typename NewMap, typename... Args>
    auto withMap(Args&&... args)
    {
        auto map = NewMap(std::forward<Args>(args)...);
        return SlamBuilder<Propagator, Backend, NewMap, Frontends...>(std::move(mPropagator), std::move(backend),
                                                                      std::move(map), std::move(mFrontends));
    }

    template <typename NewFrontend, typename... Args>
    auto withFrontend(Args&&... args)
    {
        auto frontend = NewFrontend(std::forward<Args>(args)...);

        auto appendedFrontends = std::tuple_cat(std::move(mFrontends), std::make_tuple(std::move(frontend)));

        return SlamBuilder<Propagator, Backend, Map, Frontends..., NewFrontend>(
            std::move(mPropagator), std::move(backend), std::move(mMap), std::move(appendedFrontends));
    }

    auto build() { return build<Slam>(); }

    template <template <typename...> typename SlamType>
    auto build()
    {
        static_assert(!std::is_same_v<Propagator, NoPropagator>, "Error: Missing Propagator! Call .withPropagator()");
        static_assert(!std::is_same_v<Backend, NoBackend>, "Error: Missing Backend! Call .withBackend()");
        static_assert(!std::is_same_v<Map, NoMap>, "Error: Missing Map! Call .withMap()");

        const auto unpacker = [&](auto&&... args)
        {
            return SlamType<Propagator, Backend, Map, Frontends...>(
                std::move(mPropagator), std::move(backend), std::move(mMap), std::forward<decltype(args)>(args)...);
        };

        return std::apply(unpacker, mFrontends);
    }

  private:
    SlamBuilder(Propagator propagator, Backend backend, Map map, std::tuple<Frontends...> frontends)
        : mPropagator(std::move(propagator)), backend(std::move(backend)), mMap(std::move(map)),
          mFrontends(std::move(frontends))
    {
    }

    Propagator mPropagator;
    Backend backend;
    Map mMap;
    std::tuple<Frontends...> mFrontends;
};

} // namespace modular_slam

#endif // MODULAR_SLAM_FEATURE_SLAM_BUILDER_HPP
