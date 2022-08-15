#ifndef MSLAM_VIEWER_GRID_HPP_
#define MSLAM_VIEWER_GRID_HPP_

#include <QOpenGLBuffer>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <boost/core/span.hpp>
#include <glm/vec3.hpp>
#include <qmatrix4x4.h>
#include <qopenglfunctions.h>

class GlDrawable
{
  public:
    virtual void draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view) = 0;
};

class Grid : public GlDrawable
{
  public:
    bool init();
    void draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view) override;

  private:
    bool initBuffers(const int numOfGrids);
    QOpenGLShaderProgram shader;
    QOpenGLBuffer* vertexBuffer;
    QOpenGLBuffer* indexBuffer;
};

class PointCloudDrawable : public GlDrawable
{
  public:
    bool init();
    void draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view) override;
    void setPoints(const std::vector<glm::vec3>& newPoints);

  private:
    QOpenGLShaderProgram shader;
    QOpenGLBuffer* vertexBuffer;
};

#endif // MSLAM_VIEWER_GRID_HPP_
