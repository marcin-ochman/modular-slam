#ifndef MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
#define MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_

#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <qevent.h>
#include <qpoint.h>

#include "camera.hpp"

class PointcloudViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

  public:
    PointcloudViewer(QWidget* parent = nullptr);
    ~PointcloudViewer();

  protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void handleCameraRotation(QMouseEvent* event);
    void handleCameraMovement(QMouseEvent* event);

  private:
    void initShader();
    void initBuffer();

  private:
    QOpenGLShaderProgram shader;
    QOpenGLBuffer* vertexBuffer;
    QOpenGLBuffer* indexBuffer;

    Camera camera;
    std::optional<QPoint> oldMousePosition;
    QTimer* updateTimer;
};

#endif // MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
