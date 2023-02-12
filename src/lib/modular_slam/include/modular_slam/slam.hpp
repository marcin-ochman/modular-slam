#ifndef MSLAM_SLAM_HPP_
#define MSLAM_SLAM_HPP_

#include "modular_slam/backend_interface.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"

#include <spdlog/spdlog.h>

namespace mslam
{

enum class SlamProcessResult
{
    Success,
    NoDataAvailable,
    NoConstraints,
    Error
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class Slam
{
  public:
    using DataProviderInterfaceType = DataProviderInterface<SensorDataType>;
    using FrontendInterfaceType = FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>;
    using BackendInterfaceType = BackendInterface<SensorStateType, LandmarkStateType>;
    using SlamType = Slam<SensorDataType, SensorStateType, LandmarkStateType>;
    using MapType = IMap<SensorStateType, LandmarkStateType>;

    Slam(std::shared_ptr<ParametersHandlerInterface> parameterHandler,
         std::shared_ptr<DataProviderInterfaceType> dataProviderInterface,
         std::shared_ptr<FrontendInterfaceType> frontendInterface,
         std::shared_ptr<BackendInterfaceType> backendInterface, std::shared_ptr<MapType> mapInterface)
    {
        this->parameterHandler = std::move(parameterHandler);
        this->dataProvider = std::move(dataProviderInterface);
        this->frontend = std::move(frontendInterface);
        this->backend = std::move(backendInterface);
        this->map = std::move(mapInterface);
    }

    virtual bool init();
    virtual SlamProcessResult process();
    const SensorStateType& currentState() { return state; }
    virtual ~Slam() = default;

    std::shared_ptr<FrontendInterfaceType> frontend;
    std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider;

    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    std::shared_ptr<MapType> map;
    std::shared_ptr<BackendInterfaceType> backend;

  private:
    SensorStateType state;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
bool Slam<SensorDataType, SensorStateType, LandmarkStateType>::init()
{
    auto result =
        parameterHandler->init() && dataProvider->init() && frontend->init() && backend->init() && map->init();

    return result;
}

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
SlamProcessResult Slam<SensorDataType, SensorStateType, LandmarkStateType>::process()
{
    auto isNewDataAvailable = dataProvider->fetch();

    if(!isNewDataAvailable)
        return SlamProcessResult::NoDataAvailable;

    auto sensorData = dataProvider->recentData();
    auto constraints = frontend->prepareConstraints(*sensorData);

    if(!constraints)
    {
        spdlog::error("No constraints available");
        return SlamProcessResult::NoConstraints;
    }

    state = constraints->currentState();
    backend->optimize(*constraints);
    map->update(constraints);

    return SlamProcessResult::Success;
}

} // namespace mslam

#endif // MSLAM_SLAM_HPP_
