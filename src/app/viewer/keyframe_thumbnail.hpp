#ifndef KEYFRAME_THUMBNAIL_HPP_
#define KEYFRAME_THUMBNAIL_HPP_

#include "grid.hpp"

#include <QObject>
#include <QOpenGLTexture>
#include <qimage.h>
#include <qmatrix4x4.h>

class KeyframeThumbnail : public GlDrawable, public QObject
{
  public:
    KeyframeThumbnail(QObject* parent = nullptr);
    void draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view) override;
    bool init();

    void setImage(const QImage& newImage);
    void setPose(const QMatrix4x4& newPose) { pose = newPose; }

  private:
    QOpenGLShaderProgram shader;
    QOpenGLBuffer* vertexBuffer;
    QOpenGLBuffer* indexBuffer;
    QOpenGLTexture* texture;

    QMatrix4x4 pose;
};

#endif // KEYFRAME_THUMBNAIL_HPP_
