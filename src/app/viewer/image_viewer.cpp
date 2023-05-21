
#include "image_viewer.hpp"
#include <qpixmap.h>

void ImageViewer::drawImage(const QImage& newImage)
{
    setEnabled(true);
    image = newImage;
    ui.label->clear();
    ui.label->setPixmap(QPixmap::fromImage(image));

    if(ui.autoscaleCheckbox->isChecked())
    {
        const float imageWidth = static_cast<float>(image.width());
        const float imageHeight = static_cast<float>(image.height());
        const float scrollWidth = static_cast<float>(ui.scrollArea->width());
        const float scrollHeight = static_cast<float>(ui.scrollArea->width());

        auto heightPercent = scrollHeight / imageHeight * 100.0f;
        auto widthPercent = scrollWidth / imageWidth * 100.0f;
        auto finalPercent = std::min(heightPercent, widthPercent) * 0.9f;

        scaleDrawnImage(finalPercent);
        return;
    }

    auto percentScale = static_cast<float>(ui.horizontalSlider->value());
    scaleDrawnImage(percentScale);
}

void ImageViewer::scaleDrawnImage(float percent)
{
    const float imageWidth = static_cast<float>(image.width());
    const float imageHeight = static_cast<float>(image.height());

    ui.label->setScaledContents(true);
    auto scaledWidth = static_cast<int>(imageWidth * percent / 100.0f);
    auto scaledHeight = static_cast<int>(imageHeight * percent / 100.0f);
    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void ImageViewer::initialize()
{
    ui.setupUi(this);
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
