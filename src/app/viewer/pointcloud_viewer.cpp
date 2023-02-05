#include "pointcloud_viewer.hpp"

#include <QMouseEvent>
#include <QTimer>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
#include <optional>

PointcloudViewer::PointcloudViewer(QWidget* parent) : QOpenGLWidget(parent), camera{glm::vec3(0, 0, 10)}
{
    setMouseTracking(true);
}

PointcloudViewer::~PointcloudViewer()
{
    updateTimer->stop();
}

void PointcloudViewer::setPoints(const std::vector<glm::vec3>& newPoints)
{
    pointcloud.setPoints(newPoints);
}

void PointcloudViewer::addKeyframe(const KeyframeViewData& keyframe)
{
    auto thumbnail = new KeyframeThumbnail(this);

    thumbnail->init();
    thumbnail->setImage(keyframe.image);
    thumbnail->setPose(keyframe.pose);

    keyframeThumbnails.push_back(thumbnail);
}

void PointcloudViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);

    pointcloud.init();
    grid.init();

    updateTimer = new QTimer(this);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));

    static constexpr auto msPerFrame = 20;
    updateTimer->start(msPerFrame);
}

void PointcloudViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void PointcloudViewer::paintGL()
{
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto aspectRatio = static_cast<float>(this->width()) / static_cast<float>(this->height());

    QMatrix4x4 projection, view;
    projection.setToIdentity();
    view.setToIdentity();

    projection.perspective(camera.Zoom, aspectRatio, 0.01f, 1000.f);

    auto viewMatrix = camera.getViewMatrix();
    std::copy(glm::value_ptr(viewMatrix), glm::value_ptr(viewMatrix) + 16, view.data());

    grid.draw(*this, projection, view);
    pointcloud.draw(*this, projection, view);

    for(auto thumbnail : keyframeThumbnails)
    {
        thumbnail->draw(*this, projection, view);
    }
}

void PointcloudViewer::mouseMoveEvent(QMouseEvent* event)
{
    if(event->buttons() & Qt::LeftButton)
        handleCameraRotation(event);
    else if(event->buttons() & Qt::RightButton)
        handleCameraMovement(event);
    else
        oldMousePosition = std::nullopt;
}

void PointcloudViewer::wheelEvent(QWheelEvent* event)
{
    const auto scroll = static_cast<float>(event->angleDelta().y()) / 40.0f;
    camera.processMouseScroll(scroll);
}

void PointcloudViewer::handleCameraRotation(QMouseEvent* event)
{
    auto currentPosition = event->pos();
    if(oldMousePosition)
    {
        auto offset = currentPosition - oldMousePosition.value();
        camera.processMouseMovement(static_cast<float>(offset.x()), static_cast<float>(offset.y()));
    }

    oldMousePosition = currentPosition;
}

void PointcloudViewer::handleCameraMovement(QMouseEvent* event)
{
    auto currentPosition = event->pos();
    if(oldMousePosition)
    {
        auto offset = currentPosition - oldMousePosition.value();

        const auto xoffset = static_cast<float>(offset.x()) * 0.01f;
        const auto yoffset = static_cast<float>(offset.y()) * 0.01f;

        camera.processCameraPlaneMovement(xoffset, yoffset);
    }

    oldMousePosition = currentPosition;
}
