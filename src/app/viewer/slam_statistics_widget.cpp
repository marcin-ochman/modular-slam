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

void SlamStatisticsWidget::setKeyframesCount(std::size_t count)
{
    ui.keyframesLabel->setText(QString("%1").setNum(count));
}

void SlamStatisticsWidget::setLandmarksCount(std::size_t count)
{
    ui.landmarksLabel->setText(QString("%1").setNum(count));
}

void SlamStatisticsWidget::setSlamStatistics(const SlamStatistics& stats)
{
    setFps(1000.0f / stats.msPerFrame);
    setMsPerFrame(stats.msPerFrame);
    setKeyframesCount(stats.keyframesCount);
    setLandmarksCount(stats.landmarksCount);
}
