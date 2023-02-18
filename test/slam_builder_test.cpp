#include "modular_slam/backend_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/rgbdi_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_builder.hpp"

#include <catch2/catch.hpp>
#include <catch2/trompeloeil.hpp>
#include <trompeloeil.hpp>

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

class BackendMock : public mslam::BackendInterface<mslam::slam3d::SensorState, mslam::Vector3>
{
    using BackendOutputPtrType = std::shared_ptr<mslam::BackendOutput<mslam::slam3d::SensorState, mslam::Vector3>>;
    using FrontendOutputType = mslam::FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3>;

    MAKE_MOCK1(process, BackendOutputPtrType(FrontendOutputType&), override);
};

class FrontendMock : public mslam::FrontendInterface<mslam::RgbdiFrame, mslam::slam3d::SensorState, mslam::Vector3>
{
  public:
    using RetValue = std::shared_ptr<mslam::FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3>>;
    using Arg = mslam::RgbdiFrame;

    MAKE_MOCK1(processSensorData, RetValue(const Arg& arg), override);
};

class MapMock : public mslam::IMap<mslam::slam3d::SensorState, mslam::Vector3>
{
  public:
    using FrontendOutputPtrType = std::shared_ptr<mslam::FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3>>;

    MAKE_MOCK1(update, void(const FrontendOutputPtrType constraints), override);
    MAKE_MOCK2(visit,
               void(mslam::IMapVisitor<mslam::slam3d::SensorState, mslam::slam3d::LandmarkState>&,
                    const mslam::MapVisitingParams& params),
               override);
};

SCENARIO("Building SLAM system")
{
    GIVEN("SLAM 3D Builder")
    {
        mslam::SlamBuilder<mslam::RgbdiFrame, mslam::slam3d::SensorState, mslam::Vector3> builder;

        THEN("Build Slam system using RGBDI in 3D space")
        {
            builder.addParameterHandler(std::make_shared<ParameterHandlerMock>())
                .addDataProvider(std::make_shared<RgbdiDataProviderMock>())
                .addFrontend(std::make_shared<FrontendMock>())
                .addBackend(std::make_shared<BackendMock>())
                .addMap(std::make_shared<MapMock>());

            REQUIRE(builder.build() != nullptr);
        }
    }
}
