#include "pointcloud_viewer.hpp"
#include "grid.hpp"

#include <GL/gl.h>
#include <QMouseEvent>
#include <QTimer>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/vec3.hpp>
#include <optional>

// clang-format off
static const GLfloat textureBoxVertices[] = {
     1.0f, -1.0f,  0.0f, 1.0f, 1.0f,
     1.0f,  1.0f,  0.0f, 1.0f, 0.0f,
    -1.0f,  1.0f,  0.0f, 0.0f, 0.0f,
    -1.0f, -1.0f,  0.0f, 0.0f, 1.0f,
     0.0f,  0.0f, -0.5f, 0.0f, 0.0f
};


static const GLuint indices[] = {
        0, 1, 3,
        1, 2, 3,
        0, 4, 3,
        2, 4, 3,
        1, 4, 2,
        1, 4, 0
};
// clang-format on

constexpr auto stride = sizeof(glm::vec3) + sizeof(glm::vec2);

PointcloudViewer::PointcloudViewer(QWidget* parent)
    : QOpenGLWidget(parent), camera(65.f, static_cast<float>(width()) / static_cast<float>(height()), 0.01f, 1000.f)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

PointcloudViewer::~PointcloudViewer()
{
    updateTimer->stop();
}

void PointcloudViewer::setCurrentCameraPoints(const std::vector<glm::vec3>& newPoints)
{
    cameraPointcloud.setPoints(newPoints);
}

void PointcloudViewer::setLandmarkPoints(const std::vector<glm::vec3>& newLandmarkPoints)
{
    landmarksPointcloud.setPoints(newLandmarkPoints);
}

void PointcloudViewer::setWireframeEnabled(bool wireframeEnabled)
{
    keyframes.setWireframeEnabled(wireframeEnabled);
}

void PointcloudViewer::setThumbnailEnabled(bool imageEnabled)
{
    keyframes.setThumbnailEnabled(imageEnabled);
}

void PointcloudViewer::resetCamera()
{

    camera.resetView();
}

void PointcloudViewer::addKeyframe(const KeyframeViewData& keyframe)
{
    keyframes.addKeyframe(keyframe);
}

void PointcloudViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cameraPointcloud.init();
    landmarksPointcloud.init();
    grid.init();
    keyframes.init();

    updateTimer = new QTimer(this);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));

    static constexpr auto msPerFrame = 20;
    updateTimer->start(msPerFrame);
}

void PointcloudViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    camera.setViewportSize(static_cast<float>(w), static_cast<float>(h));
}

void PointcloudViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 projection, view;
    projection.setToIdentity();
    view.setToIdentity();

    auto viewMatrix = camera.viewMatrix();
    auto projectionMatrix = camera.projectionMatrix();
    std::copy(glm::value_ptr(viewMatrix), glm::value_ptr(viewMatrix) + 16, view.data());
    std::copy(glm::value_ptr(projectionMatrix), glm::value_ptr(projectionMatrix) + 16, projection.data());

    grid.draw(*this, projection, view);
    cameraPointcloud.draw(*this, projection, view);

    glPointSize(5.f);
    landmarksPointcloud.draw(*this, projection, view);
    glPointSize(1.f);

    keyframes.draw(*this, projection, view);
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
    const auto scroll = static_cast<float>(event->angleDelta().y()) / 500.0f;
    camera.zoom(scroll);
}

void PointcloudViewer::keyPressEvent(QKeyEvent* event)
{
    switch(event->key())
    {
        case Qt::Key_A:
            camera.pan(glm::vec2(0.2f, 0.0f));
            break;
        case Qt::Key_D:
            camera.pan(glm::vec2(-0.2f, 0.0f));
            break;
        case Qt::Key_W:
            camera.move_forward(0.2f);
            break;
        case Qt::Key_S:
            camera.move_forward(-0.2f);
            break;
    }
}

void PointcloudViewer::handleCameraRotation(QMouseEvent* event)
{
    auto currentPosition = event->position();

    if(oldMousePosition)
    {
        const auto previous = glm::vec2(oldMousePosition.value().x(), oldMousePosition.value().y());
        const auto current = glm::vec2(currentPosition.x(), currentPosition.y());

        auto diff = current - previous;
        camera.rotate(diff);
    }

    oldMousePosition = currentPosition;
}

void PointcloudViewer::handleCameraMovement(QMouseEvent* event)
{
    auto currentPosition = event->position();
    if(oldMousePosition)
    {
        const auto previous = glm::vec2(oldMousePosition.value().x(), oldMousePosition.value().y());
        const auto current = glm::vec2(currentPosition.x(), currentPosition.y());
        const auto diff = (current - previous) * 0.01f;

        camera.pan(diff);
    }

    oldMousePosition = currentPosition;
}

void PointcloudViewer::KeyframesDrawable::draw(QOpenGLFunctions& gl, const QMatrix4x4& projection,
                                               const QMatrix4x4& view)
{
    vertexBuffer->bind();
    indexBuffer->bind();
    imageShader.bind();

    imageShader.enableAttributeArray("pos");
    imageShader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, stride);
    imageShader.enableAttributeArray("tex");
    imageShader.setAttributeBuffer("tex", GL_FLOAT, sizeof(glm::vec3), 2, stride);

    const auto projectionView = projection * view;

    if(viewParams.flags.test(static_cast<std::uint8_t>(ThumbnailDrawFlags::DRAW_IMAGE)))
    {
        for(auto& thumbnail : keyframes)
        {
            thumbnail->drawImage(gl, projectionView, imageShader, viewParams);
        }
    }

    imageShader.disableAttributeArray("pos");
    imageShader.disableAttributeArray("tex");
    imageShader.release();

    wireframeShader.bind();
    wireframeShader.enableAttributeArray("pos");
    wireframeShader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, stride);
    wireframeShader.setUniformValue("globalColor", viewParams.wireframeColor);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    gl.glLineWidth(2.0);
    if(viewParams.flags.test(static_cast<std::uint8_t>(ThumbnailDrawFlags::DRAW_WIREFRAME)))
    {
        for(auto& thumbnail : keyframes)
        {
            thumbnail->drawWireframe(gl, projectionView, wireframeShader, viewParams);
        }
    }

    gl.glLineWidth(1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    vertexBuffer->release();
    indexBuffer->release();
}

void PointcloudViewer::KeyframesDrawable::setWireframeEnabled(bool wireframeEnabled)
{
    setFlag(wireframeEnabled, ThumbnailDrawFlags::DRAW_WIREFRAME);
}

void PointcloudViewer::KeyframesDrawable::setThumbnailEnabled(bool imageEnabled)
{
    setFlag(imageEnabled, ThumbnailDrawFlags::DRAW_IMAGE);
}

void PointcloudViewer::KeyframesDrawable::addKeyframe(const KeyframeViewData& keyframe)
{
    auto thumbnail = std::make_unique<KeyframeFrustum>();

    thumbnail->init();
    thumbnail->setImage(keyframe.image);
    thumbnail->setPose(keyframe.pose);
    thumbnail->setId(keyframe.id);

    keyframes.push_back(std::move(thumbnail));
}

void PointcloudViewer::KeyframesDrawable::updateKeyframe(const KeyframeViewData& keyframe)
{
    auto* keyframeFrustum = findKeyframeFrustum(keyframe.id);

    if(keyframeFrustum == nullptr)
        return;

    keyframeFrustum->setImage(keyframe.image);
    keyframeFrustum->setPose(keyframe.pose);
}

void PointcloudViewer::KeyframesDrawable::updateKeyframe(mslam::Id id, const QMatrix4x4& pose)
{
    auto* keyframeFrustum = findKeyframeFrustum(id);

    if(keyframeFrustum == nullptr)
        return;

    keyframeFrustum->setPose(pose);
}

void PointcloudViewer::KeyframesDrawable::updateKeyframe(mslam::Id id, const QImage& image)
{
    auto* keyframeFrustum = findKeyframeFrustum(id);

    if(keyframeFrustum == nullptr)
        return;

    keyframeFrustum->setImage(image);
}

void PointcloudViewer::KeyframesDrawable::setFlag(bool value, ThumbnailDrawFlags flags)
{
    viewParams.flags.set(static_cast<std::size_t>(flags), value);
}

PointcloudViewer::KeyframesDrawable::KeyframeFrustum*
PointcloudViewer::KeyframesDrawable::findKeyframeFrustum(mslam::Id keyframeId)
{
    const auto endIt = std::end(keyframes);
    auto foundIt = std::find_if(std::begin(keyframes), endIt,
                                [keyframeId](auto& keyframePtr) { return keyframePtr->keyframeId() == keyframeId; });

    return foundIt != endIt ? foundIt->get() : nullptr;
}

void PointcloudViewer::setCurrentFrame(const KeyframeViewData& keyframe)
{
    static bool isInit = false;
    if(!isInit)
    {
        isInit = true;
        KeyframeViewData currentThumbnail = {0, keyframe.image, keyframe.pose};
        addKeyframe(currentThumbnail);
    }

    keyframes.updateKeyframe(0, keyframe.image);
    keyframes.updateKeyframe(0, keyframe.pose);
}

bool PointcloudViewer::KeyframesDrawable::init()
{
    imageShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/thumbnail.glsl");
    imageShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/thumbnail_fragment.glsl");
    imageShader.link();

    vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

    vertexBuffer->create();
    vertexBuffer->bind();
    vertexBuffer->setUsagePattern(QOpenGLBuffer::DynamicDraw);

    vertexBuffer->allocate(&textureBoxVertices[0], sizeof(textureBoxVertices));

    imageShader.enableAttributeArray("pos");
    imageShader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, stride);
    imageShader.enableAttributeArray("tex");
    imageShader.setAttributeBuffer("tex", GL_FLOAT, sizeof(glm::vec3), 2, stride);

    indexBuffer->create();
    indexBuffer->bind();
    indexBuffer->allocate(&indices[0], sizeof(indices));
    indexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);

    wireframeShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/wireframe.glsl");
    wireframeShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/wireframe_fragment.glsl");
    wireframeShader.link();

    vertexBuffer->release();
    indexBuffer->release();

    return true;
}

void PointcloudViewer::KeyframesDrawable::KeyframeFrustum::drawImage(QOpenGLFunctions& gl,
                                                                     const QMatrix4x4& projectionView,
                                                                     QOpenGLShaderProgram& shader,
                                                                     const ThumbnailViewParams& viewParams)
{
    if(!texture->isCreated())
        return;

    texture->bind();

    QMatrix4x4 transform;
    transform.setToIdentity();
    transform.scale(1.f, 1.f / aspectRatio);
    transform.scale(viewParams.scale);

    shader.setUniformValue("mvp", projectionView * pose * transform);
    shader.setUniformValue("ourTexture", 0);

    gl.glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    texture->release();
}

void PointcloudViewer::KeyframesDrawable::KeyframeFrustum::drawWireframe(QOpenGLFunctions& gl,
                                                                         const QMatrix4x4& projectionView,
                                                                         QOpenGLShaderProgram& shader,
                                                                         const ThumbnailViewParams& viewParams)
{
    QMatrix4x4 transform;
    transform.setToIdentity();
    transform.scale(1.f, 1.f / aspectRatio);
    transform.scale(viewParams.scale);

    shader.setUniformValue("mvp", projectionView * pose * transform);
    gl.glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, (void*)(6 * sizeof(GLuint)));
}

void PointcloudViewer::KeyframesDrawable::KeyframeFrustum::setImage(const QImage& image)
{
    aspectRatio = static_cast<float>(image.width()) / static_cast<float>(image.height());

    QMatrix4x4 transform;
    transform.setToIdentity();
    transform.scale(1.f, 1.f / aspectRatio);

    const auto mirroredImage = image.mirrored();
    texture->bind();

    if(!texture->isStorageAllocated())
    {
        texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        texture->setMagnificationFilter(QOpenGLTexture::Linear);
        texture->setAutoMipMapGenerationEnabled(true);
        texture->setData(mirroredImage);
        return;
    }

    assert(texture->width() == image.width());
    assert(texture->height() == image.height());

    texture->setData(QOpenGLTexture::BGR, QOpenGLTexture::UInt8, mirroredImage.bits());
}

bool PointcloudViewer::KeyframesDrawable::KeyframeFrustum::init()
{
    texture = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
    return texture->create();
}
