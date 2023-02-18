#ifndef SLAM_STATISTICS_WIDGET_HPP_
#define SLAM_STATISTICS_WIDGET_HPP_

#include "ui_slam_statistics.h"
#include <QWidget>

#include "slam_statistics.hpp"

class SlamStatisticsWidget : public QWidget
{
  public:
    SlamStatisticsWidget(QWidget* parent = nullptr);

    void setMsPerFrame(float msPerFrame);
    void setFps(float fps);
    void setSlamStatistics(const SlamStatistics& stats);
    void setLandmarksCount(std::size_t count);
    void setKeyframesCount(std::size_t count);

  private:
    Ui::SlamStatistics ui;
};

#endif // SLAM_STATISTICS_WIDGET_HPP_
