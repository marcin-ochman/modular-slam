#ifndef MSLAM_SLAM_BUILDER_HPP_
#define MSLAM_SLAM_BUILDER_HPP_

#include "modular_slam/slam.hpp"
#include <memory>

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class SlamBuilder
{
  public:
    using FrontendInterfaceType = FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>;
    using BackendInterfaceType = BackendInterface<SensorStateType, LandmarkStateType>;
    using SlamType = Slam<SensorDataType, SensorStateType, LandmarkStateType>;

    std::unique_ptr<SlamType> build();

    SlamBuilder& addDataProvider(std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider)
    {
        dataProviderInterface = dataProvider;
        return *this;
    }

    SlamBuilder&
    addFrontend(std::shared_ptr<FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>> frontend)
    {
        frontendInterface = frontend;
        return *this;
    }

    SlamBuilder& addBackend(std::shared_ptr<BackendInterfaceType> backend)
    {
        backendInterface = backend;
        return *this;
    }

    SlamBuilder& addMap(std::shared_ptr<MapInterface> map)
    {
        mapInterface = map;
        return *this;
    }

    SlamBuilder& addParameterHandler(std::shared_ptr<ParametersHandlerInterface> handler)
    {
        parameterHandler = handler;
        return *this;
    }

  private:
    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    std::shared_ptr<DataProviderInterface<SensorDataType>> dataProviderInterface;
    std::shared_ptr<MapInterface> mapInterface;
    std::shared_ptr<BackendInterfaceType> backendInterface;
    std::shared_ptr<FrontendInterfaceType> frontendInterface;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
std::unique_ptr<typename SlamBuilder<SensorDataType, SensorStateType, LandmarkStateType>::SlamType>
SlamBuilder<SensorDataType, SensorStateType, LandmarkStateType>::build()
{
    auto slam = std::make_unique<SlamType>(parameterHandler, dataProviderInterface, frontendInterface, backendInterface,
                                           mapInterface);
    return slam;
}

} // namespace mslam

#endif // MSLAM_SLAM_BUILDER_HPP_
