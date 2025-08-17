
#include "image_viewer.hpp"
#include <QPainter>
#include <QPixmap>

ImageViewer::ImageViewer(QWidget* parent) : QWidget(parent)
{
    ui.setupUi(this);
    connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}

QImage ImageViewer::grabImage() const
{
    return ui.label->pixmap().toImage();
}

void ImageViewer::drawImage(const QImage& newImage)
{
    setEnabled(true);
    image = newImage;

    auto pixmap = QPixmap::fromImage(image);

    drawPixmap(pixmap);
}

void ImageViewer::drawImageWithObservations(const QImage& newImage, const QVector<QObservation>& observations)
{
    setEnabled(true);
    image = newImage;
    auto pixmap = QPixmap::fromImage(image);

    QPainter paint(&pixmap);

    for(const auto& observation : observations)
    {
        const auto& keypoint = observation.keypoint;
        const auto& landmark = observation.projectedLandmark;

        constexpr auto radius = 2;
        constexpr auto size = 2 * radius;

        paint.setPen(QColor(255, 0, 0));
        paint.setBrush(QColor(255, 0, 0));
        paint.drawEllipse(keypoint.x() - radius, keypoint.y() - radius, size, size);

        paint.setPen(QColor(0, 0, 255));
        paint.setBrush(QColor(0, 0, 255));
        paint.drawEllipse(landmark.x() - radius, landmark.y() - radius, size, size);

        paint.setPen(QColor(0, 255, 0));
        paint.setBrush(QColor(0, 255, 0));

        paint.drawLine(landmark.x(), landmark.y(), keypoint.x(), keypoint.y());
    }

    drawPixmap(pixmap);
}

void ImageViewer::drawPixmap(const QPixmap& pixmap)
{
    ui.label->setPixmap(pixmap);

    if(ui.autoscaleCheckbox->isChecked())
    {
        const float imageWidth = static_cast<float>(image.width());
        const float imageHeight = static_cast<float>(image.height());
        const float scrollWidth = static_cast<float>(ui.scrollArea->width());
        const float scrollHeight = static_cast<float>(ui.scrollArea->width());

        const auto heightPercent = scrollHeight / imageHeight * 100.0f;
        const auto widthPercent = scrollWidth / imageWidth * 100.0f;
        const auto finalPercent = std::min(heightPercent, widthPercent) * 0.9f;

        scaleDrawnImage(finalPercent);
        return;
    }

    auto percentScale = static_cast<float>(ui.horizontalSlider->value());
    scaleDrawnImage(percentScale);
}

void ImageViewer::drawPointsOnImage(QPixmap& pixmap, const QVector<QPoint>& points) {}

void ImageViewer::scaleDrawnImage(float percent)
{
    const float imageWidth = static_cast<float>(image.width());
    const float imageHeight = static_cast<float>(image.height());

    ui.label->setScaledContents(true);

    const auto scaledWidth = static_cast<int>(imageWidth * percent / 100.0f);
    const auto scaledHeight = static_cast<int>(imageHeight * percent / 100.0f);

    ui.label->setFixedSize(scaledWidth, scaledHeight);
}

void ImageViewer::initialize() {}
