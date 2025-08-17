#ifndef RGBD_FILE_PROVIDER_H_
#define RGBD_FILE_PROVIDER_H_

#include "modular_slam/sensors/data_provider.hpp"
#include "modular_slam/types/rgbd_frame.hpp"

#include <chrono>
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
    std::vector<double> timestamps;
};

class RgbdFileProvider : public DataProviderInterface<RgbdFrame>
{
  public:
    RgbdFileProvider(RgbdFilePaths paths, const CameraParameters& cameraParams);

    virtual bool init() override;
    virtual bool fetch() override;
    virtual std::shared_ptr<RgbdFrame> recentData() const override;

    RgbdFileProvider() = delete;
    RgbdFileProvider(const RgbdFileProvider&) = delete;
    RgbdFileProvider(const RgbdFileProvider&&) = delete;
    RgbdFileProvider& operator=(const RgbdFileProvider&) = delete;
    RgbdFileProvider& operator=(const RgbdFileProvider&&) = delete;

  private:
    RgbdFilePaths filePaths;

    // std::vector<std::string> rgbPaths;
    // std::vector<std::string> depthPaths;
    std::size_t currentIndex = 0;
    std::shared_ptr<RgbdFrame> recentFrame;

    CameraParameters cameraParameters;
};

RgbdFilePaths readTumRgbdDataset(const std::filesystem::path& tumFile);
CameraParameters tumRgbdCameraParams();

} // namespace mslam

#endif // RGBD_FILE_PROVIDER_H_
