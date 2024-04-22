#ifndef DATA_PROVIDER_HPP_
#define DATA_PROVIDER_HPP_

#include "modular_slam/slam_component.hpp"
#include <memory>

namespace mslam
{

template <typename DataType>
class DataProviderInterface : public SlamComponent
{
  public:
    virtual bool init() = 0;
    virtual bool fetch() = 0;
    virtual std::shared_ptr<DataType> recentData() const = 0;
};

} // namespace mslam

#endif /* DATA_PROVIDER_HPP_ */
