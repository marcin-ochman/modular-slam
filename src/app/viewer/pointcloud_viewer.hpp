#ifndef MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
#define MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_

#include "camera.hpp"
#include "grid.hpp"

#include <QEvent>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QPoint>

#include <bitset>

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
    class KeyframesDrawable : public GlDrawable
    {
      public:
        void addKeyframe(const KeyframeViewData& keyframe);
        bool init();
        void draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view) override;
        void setWireframeEnabled(bool wireframeEnabled);
        void setImageEnabled(bool imageEnabled);

      private:
        enum class ThumbnailDrawFlags : std::uint8_t
        {
            DRAW_IMAGE = 0,
            DRAW_WIREFRAME
        };

        struct ThumbnailViewParams
        {
            float scale;
            std::bitset<2> flags;
            QColor wireframeColor;
        };

        void setFlag(bool value, ThumbnailDrawFlags flags);
        void drawFrustum();

        QOpenGLShaderProgram imageShader;
        QOpenGLShaderProgram wireframeShader;
        QOpenGLBuffer* vertexBuffer;
        QOpenGLBuffer* indexBuffer;

        ThumbnailViewParams viewParams = {0.25f, 3, QColor(20, 20, 255, 255)};

        class KeyframeFrustum
        {
          public:
            void drawImage(QOpenGLFunctions& gl, const QMatrix4x4& projectionView, QOpenGLShaderProgram& shader,
                           const ThumbnailViewParams& viewParams);
            void drawWireframe(QOpenGLFunctions& gl, const QMatrix4x4& projectionView, QOpenGLShaderProgram& shader,
                               const ThumbnailViewParams& viewParams);
            bool init();
            void setImage(const QImage& newImage);
            void setPose(const QMatrix4x4& newPose) { pose = newPose; }

          private:
            QOpenGLTexture* texture;
            QMatrix4x4 pose;
            float aspectRatio;
        };

        std::vector<std::unique_ptr<KeyframeFrustum>> keyframes;
    };

    Camera camera;
    std::optional<QPoint> oldMousePosition;
    QTimer* updateTimer;

    Grid grid;
    PointCloudDrawable pointcloud;
    KeyframesDrawable keyframes;
};

#endif // MSLAM_VIEWER_POINTCLOUD_VIEWER_HPP_
