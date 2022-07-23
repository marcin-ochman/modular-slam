
#include "image_viewer.hpp"

void ImageViewer::drawImage(const QPixmap& newPixmap)
{
    setEnabled(true);
    pixmap = newPixmap;
    ui.label->clear();
    ui.label->setPixmap(pixmap);
    if(ui.autoscaleCheckbox->isChecked())
    {
        auto heightPercent = static_cast<float>(ui.scrollArea->height()) / pixmap.height() * 100.0f;
        auto widthPercent = static_cast<float>(ui.scrollArea->width()) / pixmap.width() * 100.0f;
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
    auto scaledWidth = pixmap.width() * percent / 100.0f;
    auto scaledHeight = pixmap.height() * percent / 100.0f;
    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void ImageViewer::initialize()
{
    ui.setupUi(this);
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
