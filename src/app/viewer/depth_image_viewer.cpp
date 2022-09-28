#include "depth_image_viewer.hpp"
#include "modular_slam/depth_frame.hpp"
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

    const double min = minDepth(), max = maxDepth();
    cv::convertScaleAbs(depth - min, depthWithColormap, 255.0 / (max - min));
    cv::applyColorMap(depthWithColormap, depthWithColormap, cv::COLORMAP_HOT);

    image = QImage{depthWithColormap.ptr(), newDepth.size.width, newDepth.size.height, 3 * newDepth.size.width,
                   QImage::Format_BGR888}
                .copy();

    ui.label->setPixmap(QPixmap::fromImage(image));
    if(ui.autoscaleCheckbox->isChecked())
    {
        auto heightPercent = static_cast<float>(ui.scrollArea->height()) / image.height() * 100.0f;
        auto widthPercent = static_cast<float>(ui.scrollArea->width()) / image.width() * 100.0f;
        scaleDrawnImage(std::min(heightPercent, widthPercent) * 0.9f);
    }
    else
    {
        scaleDrawnImage(ui.horizontalSlider->value());
    }
}

void DepthImageViewer::scaleDrawnImage(int percent)
{
    ui.label->setScaledContents(true);
    auto scaledWidth = image.width() * percent / 100.0f;
    auto scaledHeight = image.height() * percent / 100.0f;
    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void DepthImageViewer::initialize()
{
    ui.setupUi(this);
    ui.colorbar->hide();
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
