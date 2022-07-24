#ifndef SLAM_THREAD_H_
#define SLAM_THREAD_H_

#include <QImage>
#include <QPixmap>
#include <QThread>

#include "modular_slam/depth_frame.hpp"
#include "modular_slam/realsense_camera.hpp"

class SlamThread : public QThread
{
    Q_OBJECT

  public:
    SlamThread(QObject* parent) : QThread(parent) {}

  public slots:
    void stop() { m_isRunning = false; }

  private:
    void run() override
    {
        m_isRunning = true;
        auto dataProvider = std::make_shared<mslam::RealSenseCamera>();

        dataProvider->init();

        while(m_isRunning)
        {
            if(dataProvider->fetch())
            {
                auto rgbd = dataProvider->recentData();
                QImage image{rgbd->rgb.data.data(), rgbd->rgb.size.width, rgbd->rgb.size.height,
                             3 * rgbd->rgb.size.width, QImage::Format_BGR888};

                emit newRgbImageAvailable(image);
                emit newDepthImageAvailable(rgbd->depth);
            }
        }
    }

  signals:
    void newRgbImageAvailable(const QImage& pixmap);
    void newDepthImageAvailable(const mslam::DepthFrame& depth);

  private:
    bool m_isRunning;
};

#endif // SLAM_THREAD_H_
