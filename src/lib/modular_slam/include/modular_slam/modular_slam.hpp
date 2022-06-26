#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/data_provider.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/types.hpp"

#include <any>
#include <cstdint>
#include <memory>
#include <vector>

namespace mslam
{
class Slam
{
  public:
    bool init();
    bool run();
};

template <typename DataType>
class SlamBuilder
{
  public:
    std::unique_ptr<Slam> build();

    SlamBuilder& addDataProvider();
    SlamBuilder& addFrontend();
    SlamBuilder& addBackend();
    SlamBuilder& addMap();
    SlamBuilder& addParameterHandler();

  private:
    std::shared_ptr<ParametersHandlerInterface> parameterHandler;
    // std::shared_ptr<DataProviderInterface<DataType>> dataProvider;
};

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
