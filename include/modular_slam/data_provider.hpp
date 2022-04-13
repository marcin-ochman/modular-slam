#ifndef DATA_PROVIDER_HPP_
#define DATA_PROVIDER_HPP_

#include <memory>

namespace mslam
{

template <typename DataType>
class DataProvider
{
  public:
    virtual bool init();
    virtual bool fetch();
    virtual std::shared_ptr<DataType> recentData() const;
};

} // namespace mslam

#endif /* DATA_PROVIDER_HPP_ */
