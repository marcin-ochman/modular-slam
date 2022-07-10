#include "modular_slam/rgbd_file_provider.hpp"
#include <algorithm>
#include <filesystem>
#include <regex>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mslam
{

namespace fs = std::filesystem;

bool hasImageExtension(const fs::path& filePath)
{
    constexpr std::array extensions = {".png", ".jpg", ".jpeg"};

    auto extension = filePath.extension();
    return std::any_of(std::begin(extensions), std::end(extensions),
                       [extension](auto& refExension) { return refExension == extension; });
}

std::vector<std::string> getImages(const fs::path& rootDir)
{
    std::vector<std::string> imagePaths;
    for(auto const& dirEntry : std::filesystem::directory_iterator{rootDir})
    {
        if(hasImageExtension(dirEntry.path()))
            imagePaths.push_back(dirEntry.path().string());
    }

    return imagePaths;
}

RgbdFileProvider::RgbdFileProvider(const fs::path& rgbRootDir, const fs::path& depthRootDir)
    : rgbPath{rgbRootDir}, depthPath{depthRootDir}
{
    rgbPaths = getImages(rgbRootDir);
    depthPaths = getImages(depthRootDir);
}

bool RgbdFileProvider::init()
{
    return rgbPaths.size() > 0 && rgbPaths.size() == depthPaths.size();
}

bool RgbdFileProvider::fetch()
{
    if(currentIndex >= rgbPaths.size())
    {
        recentFrame = nullptr;
        return false;
    }

    const auto& rgbImagePath = rgbPaths[currentIndex];
    const auto& depthPath = depthPaths[currentIndex];
    auto rgbMat = cv::imread(rgbImagePath);
    auto depthMat = cv::imread(depthPath, cv::IMREAD_ANYDEPTH);
    cv::Size frameSize = rgbMat.size();

    auto rgbMemorySize = frameSize.height * frameSize.width * rgbMat.channels();
    auto depthMemorySize = frameSize.height * frameSize.width * depthMat.channels();

    auto newRgbdFrame = std::make_shared<RgbdFrame>();
    newRgbdFrame->rgbData.resize(rgbMemorySize);
    newRgbdFrame->depthData.resize(depthMemorySize);

    cv::Mat rgbFrameView{frameSize.height, frameSize.width, CV_8UC3, newRgbdFrame->rgbData.data()};
    cv::Mat depthFrameView{frameSize.height, frameSize.width, CV_16UC1, newRgbdFrame->depthData.data()};

    rgbMat.copyTo(rgbFrameView);
    depthMat.copyTo(depthFrameView);

    currentIndex += 1;

    recentFrame = std::move(newRgbdFrame);

    return true;
}

std::shared_ptr<RgbdFrame> RgbdFileProvider::recentData() const
{
    return recentFrame;
}

} // namespace mslam
