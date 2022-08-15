#include "pointcloud_viewer.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <QMouseEvent>
#include <QTimer>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <optional>
#include <qevent.h>
#include <qmatrix4x4.h>
#include <qnamespace.h>
#include <qopenglwidget.h>
#include <qwidget.h>

PointcloudViewer::PointcloudViewer(QWidget* parent) : QOpenGLWidget(parent), camera{glm::vec3(0, 5, 13)}
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

void PointcloudViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);

    pointcloud.init();
    grid.init();

    updateTimer = new QTimer(this);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));
    updateTimer->start(1 / 30.f);
}

void PointcloudViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void PointcloudViewer::paintGL()
{
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto aspectRatio = static_cast<float>(this->width()) / this->height();

    QMatrix4x4 projection, view;
    projection.setToIdentity();
    view.setToIdentity();

    projection.perspective(camera.Zoom, aspectRatio, 0.01f, 1000.f);

    auto viewMatrix = camera.getViewMatrix();
    std::copy(glm::value_ptr(viewMatrix), glm::value_ptr(viewMatrix) + 16, view.data());

    grid.draw(*this, projection, view);
    pointcloud.draw(*this, projection, view);
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
    QPoint numDegrees = event->angleDelta() / 8 * 0.2;
    camera.processMouseScroll(numDegrees.y());
}

void PointcloudViewer::handleCameraRotation(QMouseEvent* event)
{
    auto currentPosition = event->pos();
    if(oldMousePosition)
    {
        auto offset = currentPosition - oldMousePosition.value();
        camera.processMouseMovement(offset.x(), offset.y());
    }

    oldMousePosition = currentPosition;
}

void PointcloudViewer::handleCameraMovement(QMouseEvent* event)
{
    auto currentPosition = event->pos();
    if(oldMousePosition)
    {
        auto offset = currentPosition - oldMousePosition.value();

        float xoffset = offset.x() * 0.01;
        float yoffset = offset.y() * 0.01;
        camera.processCameraPlaneMovement(xoffset, yoffset);
    }

    oldMousePosition = currentPosition;
}
