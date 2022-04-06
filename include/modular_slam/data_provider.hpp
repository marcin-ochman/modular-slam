#ifndef DATA_PROVIDER_HPP_
#define DATA_PROVIDER_HPP_

namespace mslam
{

template <typename DataType>
class DataProvider
{
  public:
    bool fetch();
    std::shared_ptr<DataType> recentData() const;
};

} // namespace mslam

#endif /* DATA_PROVIDER_HPP_ */
