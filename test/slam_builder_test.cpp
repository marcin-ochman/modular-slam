#include "modular_slam/slam_builder.hpp"
#include "modular_slam/types.hpp"

#include <catch2/catch.hpp>
#include <catch2/trompeloeil.hpp>

class ParameterHandlerMock : public mslam::ParametersHandlerInterface
{
  public:
    MAKE_MOCK2(registerParameter,
               bool(const mslam::ParameterDefinition& /*paramDefinition*/, const mslam::ParameterValue& /*value*/),
               override);
    // bool setParameter(const std::string& /*name*/, const ParameterValue& /*newValue*/) { return true; }
    // std::optional<ParameterValue> getParameter(const std::string& /*name*/) const { return 0; }
};

class RgbdiDataProviderMock : public mslam::DataProviderInterface<mslam::RgbdiFrame>
{
    MAKE_MOCK0(init, bool(), override);
    MAKE_MOCK0(fetch, bool(), override);
    MAKE_MOCK0(recentData, std::shared_ptr<mslam::RgbdiFrame>(), const override);
};

class BackendMock : public mslam::BackendInterface<mslam::slam3d::State, mslam::Vector3d>
{
};

class FrontendMock : public mslam::FrontendInterface<mslam::slam3d::State>
{
};

class MapMock : public mslam::MapInterface
{
};

SCENARIO("Building SLAM system")
{
    GIVEN("SLAM 3D Builder")
    {
        mslam::SlamBuilder<mslam::RgbdiFrame, mslam::slam3d::State, mslam::Vector3d> builder;

        THEN("Build Slam system using RGBDI in 3D space")
        {
            builder.addParameterHandler(std::make_shared<ParameterHandlerMock>())
                .addDataProvider(std::make_shared<RgbdiDataProviderMock>())
                .addBackend(std::make_shared<BackendMock>())
                .addFrontend(std::make_shared<FrontendMock>())
                .addMap(std::make_shared<MapMock>());

            REQUIRE(builder.build() != nullptr);
        }
    }
}
