#ifndef RGBD_FILE_PROVIDER_H_
#define RGBD_FILE_PROVIDER_H_

#include "modular_slam/data_provider.hpp"
#include "modular_slam/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace mslam
{

class RgbdFileProvider : public DataProvider<RgbdFrame>
{

  public:
    RgbdFileProvider(const std::filesystem::path& rgbRootDir, const std::filesystem::path& depthRootDir);
    virtual bool init() override;
    virtual bool fetch() override;
    virtual std::shared_ptr<RgbdFrame> recentData() const override;

  private:
    std::filesystem::path rgbPath;
    std::filesystem::path depthPath;
    std::vector<std::string> rgbPaths;
    std::vector<std::string> depthPaths;

    std::size_t currentIndex = 0;

    std::shared_ptr<RgbdFrame> recentFrame;
};
} // namespace mslam

#endif // RGBD_FILE_PROVIDER_H_
