#ifndef MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
#define MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_

#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>

class PointcloudViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

  public:
    PointcloudViewer(QWidget* parent = nullptr) : QOpenGLWidget(parent) {}

    ~PointcloudViewer();

  protected:
    void initializeGL();         // initialization OpenGL
    void resizeGL(int w, int h); // adjustment oeenGL The display window of
    void paintGL();              // draw opengl Images
  private:
    void InitShader();
    void InitBuffer();

  private:
    QOpenGLShaderProgram shader;
    QOpenGLBuffer* vertexBuffer;
    QOpenGLBuffer* indexBuffer;
    float viewPortWidth = 300;
    float viewPortHeight = 300;
};

#endif // MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
