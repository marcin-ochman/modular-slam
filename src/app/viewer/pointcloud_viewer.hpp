#ifndef MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
#define MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_

#include "camera.hpp"
#include "grid.hpp"
#include "keyframe_thumbnail.hpp"

#include <QEvent>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPoint>

struct KeyframeViewData
{
    QImage image;
    QMatrix4x4 pose;
};

class PointcloudViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

  public:
    PointcloudViewer(QWidget* parent = nullptr);
    ~PointcloudViewer();

  public slots:
    void setPoints(const std::vector<glm::vec3>& newPoints);
    void addKeyframe(const KeyframeViewData& keyframe);

  protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void handleCameraRotation(QMouseEvent* event);
    void handleCameraMovement(QMouseEvent* event);

  private:
    Camera camera;
    std::optional<QPoint> oldMousePosition;
    QTimer* updateTimer;

    Grid grid;
    PointCloudDrawable pointcloud;
    std::vector<KeyframeThumbnail*> keyframeThumbnails;
};

#endif // MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
