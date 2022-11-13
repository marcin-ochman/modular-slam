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

  private:
    Ui::SlamStatistics ui;
};

#endif // SLAM_STATISTICS_WIDGET_HPP_
