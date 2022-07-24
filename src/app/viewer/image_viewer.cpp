
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
        auto heightPercent = static_cast<float>(ui.scrollArea->height()) / image.height() * 100.0f;
        auto widthPercent = static_cast<float>(ui.scrollArea->width()) / image.width() * 100.0f;
        scaleDrawnImage(std::min(heightPercent, widthPercent) * 0.9f);
    }
    else
    {
        scaleDrawnImage(ui.horizontalSlider->value());
    }
}

void ImageViewer::scaleDrawnImage(int percent)
{
    ui.label->setScaledContents(true);
    auto scaledWidth = image.width() * percent / 100.0f;
    auto scaledHeight = image.height() * percent / 100.0f;
    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void ImageViewer::initialize()
{
    ui.setupUi(this);
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
