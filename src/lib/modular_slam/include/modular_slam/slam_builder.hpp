#ifndef MSLAM_SLAM_BUILDER_HPP_
#define MSLAM_SLAM_BUILDER_HPP_

#include "modular_slam/data_provider.hpp"
#include "modular_slam/slam.hpp"
#include <boost/signals2/signal.hpp>
#include <memory>

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class SlamBuilder
{
    class DataProviderWithActions : public DataProviderInterface<SensorDataType>
    {
      public:
        explicit DataProviderWithActions(std::shared_ptr<DataProviderInterface<SensorDataType>> baseDataProvider)
            : dataProvider(std::move(baseDataProvider))
        {
        }

        template <typename Callable>
        void registerDataFetchedAction(Callable&& action)
        {
            dataFetchedSignal.connect(action);
        }

        bool init() override { return dataProvider->init(); }
        std::shared_ptr<SensorDataType> recentData() const override { return dataProvider->recentData(); };
        bool fetch() override
        {
            const auto result = dataProvider->fetch();
            dataFetchedSignal(recentData());
            return result;
        }

      private:
        std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider;
        boost::signals2::signal<void(std::shared_ptr<SensorDataType>)> dataFetchedSignal;
    };

  public:
    using FrontendInterfaceType =
        FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>;
    using BackendInterfaceType = BackendInterface<SensorStateType, LandmarkStateType, ObservationType>;
    using MapType = IMap<SensorStateType, LandmarkStateType, ObservationType>;
    using SlamType = Slam<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>;

    std::unique_ptr<SlamType> build();

    SlamBuilder& addDataProvider(std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider)
    {
        auto dataProviderWithActions = std::make_shared<DataProviderWithActions>(std::move(dataProvider));
        dataProviderInterface = dataProviderWithActions;
        return *this;
    }

    SlamBuilder& addFrontend(std::shared_ptr<FrontendInterfaceType> frontend)
    {
        frontendInterface = std::move(frontend);
        frontendInterface->setParameterHandler(parameterHandler);

        return *this;
    }

    SlamBuilder& addBackend(std::shared_ptr<BackendInterfaceType> backend)
    {
        backendInterface = std::move(backend);
        backendInterface->setParameterHandler(parameterHandler);
        return *this;
    }

    SlamBuilder& addMap(std::shared_ptr<MapType> map)
    {
        mapInterface = std::move(map);
        mapInterface->setParameterHandler(parameterHandler);

        return *this;
    }

    SlamBuilder& addParameterHandler(std::shared_ptr<ParametersHandlerInterface> handler)
    {
        parameterHandler = handler;
        return *this;
    }

    // TODO: add more register methods
    template <typename Callable>
    SlamBuilder& registerDataFetchedAction(Callable&& action)
    {
        if(dataProviderInterface == nullptr)
        {
            spdlog::error("Cannot add data fetched action. Add data provider first!");
            return *this;
        }

        dataProviderInterface->registerDataFetchedAction(action);

        return *this;
    }

  private:
    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    std::shared_ptr<DataProviderWithActions> dataProviderInterface;
    std::shared_ptr<MapType> mapInterface;
    std::shared_ptr<BackendInterfaceType> backendInterface;
    std::shared_ptr<FrontendInterfaceType> frontendInterface;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
std::unique_ptr<typename SlamBuilder<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>::SlamType>
SlamBuilder<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>::build()
{
    auto slam = std::make_unique<SlamType>(parameterHandler, dataProviderInterface, frontendInterface, backendInterface,
                                           mapInterface);
    return slam;
}

} // namespace mslam

#endif // MSLAM_SLAM_BUILDER_HPP_
