#ifndef MSLAM_SLAM_HPP_
#define MSLAM_SLAM_HPP_

#include "modular_slam/backend_interface.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class Slam
{
  public:
    using DataProviderInterfaceType = DataProviderInterface<SensorDataType>;
    using FrontendInterfaceType = FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>;
    using BackendInterfaceType = BackendInterface<SensorStateType, LandmarkStateType>;
    using SlamType = Slam<SensorDataType, SensorStateType, LandmarkStateType>;
    using MapType = MapInterface<SensorStateType, LandmarkStateType>;

    Slam(std::shared_ptr<ParametersHandlerInterface> parameterHandler,
         std::shared_ptr<DataProviderInterfaceType> dataProviderInterface,
         std::shared_ptr<FrontendInterfaceType> frontendInterface,
         std::shared_ptr<BackendInterfaceType> backendInterface, std::shared_ptr<MapType> mapInterface)
    {
        this->parameterHandler = std::move(parameterHandler);
        this->dataProviderInterface = std::move(dataProviderInterface);
        this->frontendInterface = std::move(frontendInterface);
        this->backendInterface = std::move(backendInterface);
        this->mapInterface = std::move(mapInterface);
    }

    virtual bool init();
    virtual bool process();

  protected:
    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    std::shared_ptr<DataProviderInterface<SensorDataType>> dataProviderInterface;
    std::shared_ptr<MapType> mapInterface;
    std::shared_ptr<BackendInterfaceType> backendInterface;
    std::shared_ptr<FrontendInterfaceType> frontendInterface;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
bool Slam<SensorDataType, SensorStateType, LandmarkStateType>::init()
{
    // parameterHandler->;
    // dataProviderInterface->init();
    // mapInterface->init();
    // backendInterface->init();
    // frontendInterface->init();

    return true;
}

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
bool Slam<SensorDataType, SensorStateType, LandmarkStateType>::process()
{

    return true;
}

} // namespace mslam

#endif // MSLAM_SLAM_HPP_
