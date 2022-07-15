#include "image_viewer.hpp"

void ImageViewer::drawImage(const QPixmap& pixmap)
{
    setEnabled(true);
    m_pixmap = pixmap;
    m_ui.label_2->clear();
    m_ui.label_2->setPixmap(pixmap);
    scaleDrawnImage(m_ui.horizontalSlider->value());
}

void ImageViewer::scaleDrawnImage(int percent)
{
    m_ui.label_2->setScaledContents(true);
    auto scaledWidth = m_pixmap.width() * percent / 100.0;
    auto scaledHeight = m_pixmap.height() * percent / 100.0;

    m_ui.label_2->setFixedSize(scaledWidth, scaledHeight);
}

void ImageViewer::initialize()
{
    m_ui.setupUi(this);
    connect(m_ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(scaleDrawnImage(int)));
}
