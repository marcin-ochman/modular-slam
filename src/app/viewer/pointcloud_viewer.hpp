#ifndef MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
#define MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_

#include "camera.hpp"
#include "grid.hpp"

#include <QEvent>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>

class PointcloudViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

  public:
    PointcloudViewer(QWidget* parent = nullptr);
    ~PointcloudViewer();
  public slots:
    void setPoints(const std::vector<glm::vec3>& newPoints);

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
};

#endif // MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
