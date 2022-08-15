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

    std::sort(std::begin(imagePaths), std::end(imagePaths),
              [](const std::string& a, const std::string& b) { return a < b; });

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
    cv::Size depthSize = rgbMat.size();

    auto rgbMemorySize = frameSize.height * frameSize.width * rgbMat.channels();
    auto depthMemorySize = depthSize.height * depthSize.width * depthMat.channels();

    auto newRgbdFrame = std::make_shared<RgbdFrame>();
    newRgbdFrame->rgb.data.resize(rgbMemorySize);
    newRgbdFrame->rgb.size.width = frameSize.width;
    newRgbdFrame->rgb.size.height = frameSize.height;
    newRgbdFrame->depth.data.resize(depthMemorySize);
    newRgbdFrame->depth.size.width = depthSize.width;
    newRgbdFrame->depth.size.height = depthSize.height;

    cv::Mat rgbFrameView{frameSize.height, frameSize.width, CV_8UC3, newRgbdFrame->rgb.data.data()};
    cv::Mat depthFrameView{frameSize.height, frameSize.width, CV_16UC1, newRgbdFrame->depth.data.data()};

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
