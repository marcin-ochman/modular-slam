#include "slam_statistics_widget.hpp"

SlamStatisticsWidget::SlamStatisticsWidget(QWidget* parent) : QWidget(parent)
{
    ui.setupUi(this);
}

void SlamStatisticsWidget::setMsPerFrame(float msPerFrame)
{
    ui.timePerFrameLabel->setText(QString().setNum(msPerFrame, 'f', 3));
}

void SlamStatisticsWidget::setFps(float fps)
{
    ui.fpsLabel->setText(QString("%1f").setNum(fps, 'f', 3));
}

void SlamStatisticsWidget::setSlamStatistics(const SlamStatistics& stats)
{
    setFps(1000.0f / stats.msPerFrame);
    setMsPerFrame(stats.msPerFrame);
}
