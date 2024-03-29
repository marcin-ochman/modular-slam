#include "modular_slam/rgbd_file_provider.hpp"
#include "modular_slam/camera_parameters.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>

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

RgbdFileProvider::RgbdFileProvider(RgbdFilePaths paths, const CameraParameters& params)
{
    filePaths = paths;

    // rgbPaths = std::move(paths.rgbPaths);
    // depthPaths = std::move(paths.depthPaths);
    cameraParameters = params;
}

bool RgbdFileProvider::init()
{
    return filePaths.rgbPaths.size() > 0 && filePaths.rgbPaths.size() == filePaths.depthPaths.size();
}

bool RgbdFileProvider::fetch()
{
    if(currentIndex >= filePaths.rgbPaths.size())
    {
        recentFrame = nullptr;
        return false;
    }

    const auto& rgbImagePath = filePaths.rgbPaths[currentIndex];
    const auto& depthPath = filePaths.depthPaths[currentIndex];
    const auto& timestamp = filePaths.timestamps[currentIndex];
    auto rgbMat = cv::imread(rgbImagePath);
    auto depthMat = cv::imread(depthPath, cv::IMREAD_ANYDEPTH);

    if(rgbMat.empty() || depthMat.empty())
    {
        recentFrame = nullptr;
        return false;
    }

    cv::Size frameSize = rgbMat.size();
    cv::Size depthSize = depthMat.size();

    auto rgbMemorySize = static_cast<std::size_t>(frameSize.height * frameSize.width * rgbMat.channels());
    auto depthMemorySize = static_cast<std::size_t>(depthSize.height * depthSize.width * depthMat.channels());

    auto newRgbdFrame = std::make_shared<RgbdFrame>();
    newRgbdFrame->timestamp.timePoint = timestamp;
    newRgbdFrame->rgb.data.resize(rgbMemorySize);
    newRgbdFrame->rgb.size.width = frameSize.width;
    newRgbdFrame->rgb.size.height = frameSize.height;
    newRgbdFrame->depth.data.resize(depthMemorySize);
    newRgbdFrame->depth.size.width = depthSize.width;
    newRgbdFrame->depth.size.height = depthSize.height;
    newRgbdFrame->depth.cameraParameters = cameraParameters;

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

RgbdFilePaths readTumRgbdDataset(const std::filesystem::path& tumFile)
{
    std::ifstream ifss{tumFile};
    RgbdFilePaths paths;

    auto rootDir = tumFile.parent_path();

    std::string line;
    while(std::getline(ifss, line))
    {
        std::istringstream iss{line};
        std::string rgbPath, depthPath, temp;
        double timestamp;

        iss >> timestamp >> rgbPath >> temp >> depthPath;

        if(!iss.fail())
        {
            paths.rgbPaths.push_back(rootDir / rgbPath);
            paths.depthPaths.push_back(rootDir / depthPath);
            paths.timestamps.push_back(timestamp);
        }
    }

    return paths;
}

CameraParameters tumRgbdCameraParams()
{
    CameraParameters cameraParams;

    cameraParams.focal = {525, 525};
    cameraParams.principalPoint = {319.5f, 239.5f};
    cameraParams.factor = 1.f / 5000.f;

    return cameraParams;
}

} // namespace mslam
