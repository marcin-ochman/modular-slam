#include "depth_image_viewer.hpp"
#include <QPainter>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

void DepthImageViewer::drawImage(const mslam::DepthFrame& newDepth)
{
    setEnabled(true);
    ui.label->clear();

    cv::Mat depthWithColormap;
    const cv::Mat depth(newDepth.size.height, newDepth.size.width, CV_16UC1,
                        const_cast<std::uint16_t*>(newDepth.data.data()));

    const auto factor = newDepth.cameraParameters.factor;
    const double min = minDepth() / factor, max = maxDepth() / factor;
    cv::convertScaleAbs(depth - min, depthWithColormap, 255.0 / (max - min));
    cv::applyColorMap(depthWithColormap, depthWithColormap, cv::COLORMAP_HOT);

    image = QImage{depthWithColormap.ptr(), newDepth.size.width, newDepth.size.height, 3 * newDepth.size.width,
                   QImage::Format_BGR888}
                .copy();

    ui.label->setPixmap(QPixmap::fromImage(image));
    if(ui.autoscaleCheckbox->isChecked())
    {
        const float scrollHeight = static_cast<float>(ui.scrollArea->height());
        const float imageHeight = static_cast<float>(image.height());
        const float scrollWidth = static_cast<float>(ui.scrollArea->width());
        const float imageWidth = static_cast<float>(image.width());

        auto heightPercent = scrollHeight / imageHeight * 100.0f;
        auto widthPercent = scrollWidth / imageWidth * 100.0f;
        scaleDrawnImage(std::min(heightPercent, widthPercent) * 0.9f);
    }
    else
    {
        const float scale = static_cast<float>(ui.horizontalSlider->value());
        scaleDrawnImage(scale);
    }
}

void DepthImageViewer::scaleDrawnImage(float percent)
{
    ui.label->setScaledContents(true);
    const float imageWidth = static_cast<float>(image.width());
    const float imageHEight = static_cast<float>(image.height());

    auto scaledWidth = static_cast<int>(imageWidth * percent / 100.0f);
    auto scaledHeight = static_cast<int>(imageHEight * percent / 100.0f);
    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void DepthImageViewer::initialize()
{
    ui.setupUi(this);
    ui.colorbar->hide();
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
