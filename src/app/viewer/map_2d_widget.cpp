#include "map_2d_widget.hpp"
#include <qboxlayout.h>
#include <qwidget.h>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

MapWidget::MapWidget(QWidget* parent) : QWidget(parent)
{
    m_keyframeSeries = new QScatterSeries();

    QChart* chart = new QChart();
    chart->addSeries(m_keyframeSeries);
    chart->createDefaultAxes();

    auto xAxis = chart->axes(Qt::Horizontal).first();
    auto yAxis = chart->axes(Qt::Vertical).first();

    xAxis->setRange(-5, 5);
    yAxis->setRange(-5, 5);

    xAxis->setTitleText("x [m]");
    yAxis->setTitleText("y [m]");

    QChartView* chartView = new QChartView(chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);

    auto horizontalLayout = new QHBoxLayout(this);

    horizontalLayout->addWidget(chartView);
}

void MapWidget::addKeyframe(int x, int y)
{
    m_keyframeSeries->append(x, y);
}
