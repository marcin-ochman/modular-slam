#ifndef SLAM_THREAD_H_
#define SLAM_THREAD_H_

#include <QImage>
#include <QPixmap>
#include <QThread>
#include <QVector3D>
#include <QVector>
#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

#include "modular_slam/depth_frame.hpp"
#include "modular_slam/realsense_camera.hpp"

#include <QDebug>

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

                std::vector<glm::vec3> points;

                const auto& depthFrame = rgbd->depth;
                const auto xScale = 1 / depthFrame.cameraParameters.focal.x();
                const auto yScale = 1 / depthFrame.cameraParameters.focal.y();

                for(auto u = 0; u < depthFrame.size.width; ++u)
                {
                    for(auto v = 0; v < depthFrame.size.height; ++v)
                    {
                        const auto depth = depthFrame.data[v * depthFrame.size.width + u];

                        if(depth > 0)
                        {
                            const float z = depth * 0.001f;
                            const auto x = (u - depthFrame.cameraParameters.principalPoint.x()) * z * xScale;
                            const auto y = (v - depthFrame.cameraParameters.principalPoint.y()) * z * yScale;

                            const auto index = v * 3 * depthFrame.size.width + 3 * u;
                            const auto r = rgbd->rgb.data[index + 2] / 255.f;
                            const auto g = rgbd->rgb.data[index + 1] / 255.f;
                            const auto b = rgbd->rgb.data[index] / 255.f;

                            glm::vec3 point{x, y, z};
                            glm::vec3 rgb{r, g, b};

                            points.push_back(point);
                            points.push_back(rgb);
                        }
                    }
                }

                emit newPointsAvailable(points);
            }
        }
    }

  signals:
    void newRgbImageAvailable(const QImage& pixmap);
    void newDepthImageAvailable(const mslam::DepthFrame& depth);
    void newPointsAvailable(const std::vector<glm::vec3>& points);

  private:
    bool m_isRunning;
};

#endif // SLAM_THREAD_H_
