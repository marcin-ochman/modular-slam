#include "colorbar.hpp"

#include <QPainter>
#include <cmath>

ColorBar::ColorBar(QWidget* parent) : QWidget(parent) {}

void ColorBar::paintEvent(QPaintEvent* /*event*/)
{
    // QPainter painter{this};

    // QRect rect{static_cast<int>(0.4f * width()), static_cast<int>(0.1f * height()), static_cast<int>(0.2f * width()),
    //            static_cast<int>(0.8f * height())};
    // painter.setBrush(QBrush(Qt::yellow));
    // painter.fillRect(rect, painter.brush());
}
