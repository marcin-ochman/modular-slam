#include "pointcloud_viewer.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale
#include <glm/gtc/type_ptr.hpp>

#include <QMouseEvent>
#include <QTimer>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/vec3.hpp>   // glm::vec3
#include <glm/vec4.hpp>   // glm::vec4
#include <optional>
#include <qevent.h>
#include <qmatrix4x4.h>
#include <qnamespace.h>
#include <qopenglwidget.h>
#include <qwidget.h>

QString vertexShaderSource = "#version 330 core\n\
in vec3 aPos;\n\
uniform mat4 projection;\n\
uniform mat4 model;\n\
uniform mat4 view;\n\
void main()\n\
{\n\
gl_Position = projection * view * model * vec4(aPos, 1.0);\n\
}";
QString fragmentShaderSource = "#version 330 core\n\
out vec4 FragColor;\n\
void main()\n\
{\n\
FragColor = vec4(0.0f, 0.0f, 1.0f, 1.0f);\n\
}";

PointcloudViewer::PointcloudViewer(QWidget* parent) : QOpenGLWidget(parent), camera{glm::vec3(0, 5, 13)}
{
    setMouseTracking(true);
}

PointcloudViewer::~PointcloudViewer()
{
    delete vertexBuffer;
    delete indexBuffer;

    updateTimer->stop();
}

void PointcloudViewer::initializeGL()
{
    initializeOpenGLFunctions();
    initShader();
    initBuffer();

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
    // glViewport(0, 0, this->width(), this->height());
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto aspectRatio = static_cast<float>(this->width()) / this->height();
    QMatrix4x4 projection, view, model;
    projection.setToIdentity();
    view.setToIdentity();
    model.setToIdentity();
    projection.perspective(camera.Zoom, aspectRatio, 0.01f, 100000.f);

    auto viewMatrix = camera.getViewMatrix();
    std::copy(glm::value_ptr(viewMatrix), glm::value_ptr(viewMatrix) + 16, view.data());

    indexBuffer->bind();
    vertexBuffer->bind();
    shader.bind();
    shader.enableAttributeArray("aPos");
    shader.setUniformValue("projection", projection);
    shader.setUniformValue("model", model);
    shader.setUniformValue("view", view);

    glDrawElements(GL_LINES, 800, GL_UNSIGNED_INT, 0);

    shader.disableAttributeArray("aPos");
    shader.release();
    indexBuffer->release();
    vertexBuffer->release();
}
void PointcloudViewer::initShader()
{
    shader.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    shader.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    assert(shader.link());
}

void PointcloudViewer::initBuffer()
{
    vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

    std::vector<glm::vec3> vertices;
    std::vector<glm::uvec4> indices;

    auto slices = 10;
    for(int j = 0; j <= slices; ++j)
    {
        for(int i = 0; i <= slices; ++i)
        {
            float x = i - slices / 2;
            float z = -(j - slices / 2);
            float y = 0;
            vertices.push_back(glm::vec3(x, y, z));
        }
    }

    for(int j = 0; j < slices; ++j)
    {
        for(int i = 0; i < slices; ++i)
        {
            int row1 = j * (slices + 1);
            int row2 = (j + 1) * (slices + 1);

            indices.push_back(glm::uvec4(row1 + i, row1 + i + 1, row1 + i + 1, row2 + i + 1));
            indices.push_back(glm::uvec4(row2 + i + 1, row2 + i, row2 + i, row1 + i));
        }
    }

    shader.bind();
    vertexBuffer->create();
    vertexBuffer->bind();
    vertexBuffer->allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
    vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    shader.setAttributeBuffer("aPos", GL_FLOAT, 0, 3);
    vertexBuffer->release();
    shader.release();

    indexBuffer->create();
    indexBuffer->bind();
    indexBuffer->allocate(indices.data(), sizeof(indices[0]) * indices.size());
    indexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    indexBuffer->release();
}

void PointcloudViewer::mouseMoveEvent(QMouseEvent* event)
{

    if(event->buttons() & Qt::LeftButton)
    {
        handleCameraRotation(event);
    }
    else if(event->buttons() & Qt::RightButton)
    {
        handleCameraMovement(event);
    }
    else
    {
        oldMousePosition = std::nullopt;
    }
}

void PointcloudViewer::wheelEvent(QWheelEvent* event)
{
    QPoint numDegrees = event->angleDelta() / 8;
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
