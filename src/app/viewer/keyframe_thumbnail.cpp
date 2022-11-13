#include "keyframe_thumbnail.hpp"
#include <GL/gl.h>
#include <glm/vec2.hpp>

static const float textureBoxVertices[] = {
    // clang-format off
     0.5f,  0.5f,  0.0f, 1.0f, 1.0f,
     0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
    -0.5f, -0.5f,  0.0f, 0.0f, 0.0f,
    -0.5f,  0.5f,  0.0f, 0.0f, 1.0f
    // clang-format on
};

static const unsigned int indices[] = {
    // clang-format off
        0, 1, 3,
        1, 2, 3
    // clang-format on
};

KeyframeThumbnail::KeyframeThumbnail(QObject* parent) : QObject(parent)
{
    pose.setToIdentity();
}

void KeyframeThumbnail::draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view)
{
    constexpr auto stride = sizeof(glm::vec3) + sizeof(glm::vec2);

    if(!texture->isCreated())
        return;

    vertexBuffer->bind();
    indexBuffer->bind();
    shader.bind();
    texture->bind();

    shader.enableAttributeArray("pos");
    shader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, stride);
    shader.enableAttributeArray("tex");
    shader.setAttributeBuffer("tex", GL_FLOAT, sizeof(glm::vec3), 2, stride);
    shader.setUniformValue("mvp", projection * view * pose);
    shader.setUniformValue("ourTexture", 0);

    gl.glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    shader.disableAttributeArray("pos");
    shader.disableAttributeArray("tex");

    shader.release();
    vertexBuffer->release();
    indexBuffer->release();
    texture->release();
}

bool KeyframeThumbnail::init()
{
    shader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/thumbnail.glsl");
    shader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/thumbnail_fragment.glsl");
    shader.link();

    vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
    texture->create();

    vertexBuffer->create();
    vertexBuffer->bind();
    vertexBuffer->setUsagePattern(QOpenGLBuffer::DynamicDraw);

    vertexBuffer->allocate(&textureBoxVertices[0], sizeof(textureBoxVertices));
    vertexBuffer->release();

    indexBuffer->create();
    indexBuffer->bind();
    indexBuffer->allocate(&indices[0], sizeof(indices));
    indexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    indexBuffer->release();

    return true;
}

void KeyframeThumbnail::setImage(const QImage& image)
{
    texture->setData(image.mirrored());
}
