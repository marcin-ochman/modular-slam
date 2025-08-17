#ifndef MSLAM_SLAM_BUILDER_HPP_
#define MSLAM_SLAM_BUILDER_HPP_

#include "modular_slam/sensors/data_provider.hpp"
#include "modular_slam/slam.hpp"
#include <boost/signals2/signal.hpp>
#include <memory>
#include <utility>

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class SlamBuilder
{
  public:
    using FrontendInterfaceType =
        FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>;
    using BackendInterfaceType = BackendInterface<SensorStateType, LandmarkStateType, ObservationType>;
    using MapType = IMap<SensorStateType, LandmarkStateType, ObservationType>;
    using SlamType = KeypointSlam<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>;

  private:
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
            if(result)
                dataFetchedSignal(recentData());

            return result;
        }

      private:
        std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider;
        boost::signals2::signal<void(std::shared_ptr<SensorDataType>)> dataFetchedSignal;
    };

    class FrontendWithActions : public FrontendInterfaceType
    {
      public:
        explicit FrontendWithActions(std::shared_ptr<FrontendInterfaceType> baseFrontend)
            : frontend(std::move(baseFrontend))
        {
        }

        bool init() override { return frontend->init(); }

        template <typename Callable>
        void registerFrontendFinishedAction(Callable&& action)
        {
            frontendFinishedSignal.connect(action);
        }

        typename FrontendInterfaceType::FrontendOutputType
        processSensorData(std::shared_ptr<SensorDataType> sensorData) override
        {
            auto result = frontend->processSensorData(sensorData);

            frontendFinishedSignal(result);

            return result;
        }

        void update(const typename FrontendInterfaceType::BackendOutputType& backendOutput) override
        {
            frontend->update(backendOutput);
        }

      private:
        std::shared_ptr<FrontendInterfaceType> frontend;
        boost::signals2::signal<void(const typename FrontendInterfaceType::FrontendOutputType&)> frontendFinishedSignal;
    };

  public:
    std::unique_ptr<SlamType> build();

    SlamBuilder& addDataProvider(std::shared_ptr<DataProviderInterface<SensorDataType>> dataProvider)
    {
        auto dataProviderWithActions = std::make_shared<DataProviderWithActions>(std::move(dataProvider));
        dataProviderInterface = dataProviderWithActions;
        return *this;
    }

    SlamBuilder& addFrontend(std::shared_ptr<FrontendInterfaceType> frontend)
    {
        frontend->setParameterHandler(parameterHandler);

        auto frontendWithActions = std::make_shared<FrontendWithActions>(std::move(frontend));
        frontendInterface = frontendWithActions;
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

    template <typename Callable>
    SlamBuilder& registerFrontendFinishedAction(Callable&& action)
    {
        if(frontendInterface == nullptr)
        {
            spdlog::error("Cannot add frontend finished action. Add frontend first!");
            return *this;
        }

        frontendInterface->registerFrontendFinishedAction(action);

        return *this;
    }

  private:
    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    std::shared_ptr<DataProviderWithActions> dataProviderInterface;
    std::shared_ptr<MapType> mapInterface;
    std::shared_ptr<BackendInterfaceType> backendInterface;
    std::shared_ptr<FrontendWithActions> frontendInterface;
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
