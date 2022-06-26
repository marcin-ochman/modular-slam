#ifndef FACTORY_HPP_
#define FACTORY_HPP_

#include <memory>

namespace mslam
{

class Slam;

template <typename SensorType>
class SlamBuilder
{
  public:
    SlamBuilder& addMapBlock(std::unique_ptr<mslam::Map> ptr);
    SlamBuilder& addDataProvider();

    std::unique_ptr<Slam> build();
};
} // namespace mslam

#endif // FACTORY_HPP_
