#ifndef RGBD_FILE_PROVIDER_H_
#define RGBD_FILE_PROVIDER_H_

#include "modular_slam/data_provider.hpp"
#include "modular_slam/rgbd_frame.hpp"

#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

namespace mslam
{

struct RgbdFilePaths
{
    std::vector<std::string> rgbPaths;
    std::vector<std::string> depthPaths;
};

class RgbdFileProvider : public DataProviderInterface<RgbdFrame>
{
  public:
    RgbdFileProvider(const std::filesystem::path& rgbRootDir, const std::filesystem::path& depthRootDir);
    RgbdFileProvider(RgbdFilePaths paths);

    template <typename RgbIteratorType, typename DepthIteratorType>
    RgbdFileProvider(RgbIteratorType rgbBegin, RgbIteratorType rgbEnd, DepthIteratorType depthBegin,
                     DepthIteratorType depthEnd);

    virtual bool init() override;
    virtual bool fetch() override;
    virtual std::shared_ptr<RgbdFrame> recentData() const override;

    RgbdFileProvider() = delete;
    RgbdFileProvider(const RgbdFileProvider&) = delete;
    RgbdFileProvider(const RgbdFileProvider&&) = delete;
    RgbdFileProvider& operator=(const RgbdFileProvider&) = delete;
    RgbdFileProvider& operator=(const RgbdFileProvider&&) = delete;

  private:
    std::vector<std::string> rgbPaths;
    std::vector<std::string> depthPaths;

    std::size_t currentIndex = 0;

    std::shared_ptr<RgbdFrame> recentFrame;
};

template <typename RgbIteratorType, typename DepthIteratorType>
RgbdFileProvider::RgbdFileProvider(RgbIteratorType rgbBegin, RgbIteratorType rgbEnd, DepthIteratorType depthBegin,
                                   DepthIteratorType depthEnd)
{
    std::copy(rgbBegin, rgbEnd, std::back_inserter(rgbPaths));
    std::copy(depthBegin, depthEnd, std::back_inserter(depthPaths));
}

RgbdFilePaths readTumRgbdDataset(const std::filesystem::path& tumFile);

} // namespace mslam

#endif // RGBD_FILE_PROVIDER_H_
