#include "grid.hpp"

#include <GL/gl.h>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>
#include <qquaternion.h>

bool Grid::init()
{
    shader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/mvp.glsl");
    shader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fragment.glsl");
    if(!shader.link())
        return false;

    constexpr auto cellsCount = 10;
    return initBuffers(cellsCount);
}

bool Grid::initBuffers(const int cellsCount)
{
    vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

    std::vector<glm::vec3> vertices;
    std::vector<glm::uvec4> indices;
    const float cellsCountFloat = static_cast<float>(cellsCount);

    for(int j = 0; j <= cellsCount; ++j)
    {
        for(int i = 0; i <= cellsCount; ++i)
        {
            const auto x = static_cast<float>(i) - cellsCountFloat / 2.0f;
            const auto z = static_cast<float>(j) - cellsCountFloat / 2.0f;
            const auto y = 1.0f;

            vertices.push_back(glm::vec3(x, y, z));
        }
    }

    for(int j = 0; j < cellsCount; ++j)
    {
        for(int i = 0; i < cellsCount; ++i)
        {
            int firstRow = j * (cellsCount + 1);
            int secondRow = (j + 1) * (cellsCount + 1);

            indices.push_back(glm::uvec4(firstRow + i, firstRow + i + 1, firstRow + i + 1, secondRow + i + 1));
            indices.push_back(glm::uvec4(secondRow + i + 1, secondRow + i, secondRow + i, firstRow + i));
        }
    }

    vertexBuffer->create();
    vertexBuffer->bind();
    const auto vertexSize = static_cast<int>(sizeof(vertices[0]) * vertices.size());
    vertexBuffer->allocate(vertices.data(), vertexSize);
    vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vertexBuffer->release();

    indexBuffer->create();
    indexBuffer->bind();
    const auto indicesSize = static_cast<int>(sizeof(indices[0]) * indices.size());
    indexBuffer->allocate(indices.data(), indicesSize);
    indexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    indexBuffer->release();

    return true;
}

void Grid::draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view)
{
    QMatrix4x4 model;
    model.setToIdentity();
    indexBuffer->bind();
    vertexBuffer->bind();
    shader.bind();
    shader.enableAttributeArray("pos");

    shader.setAttributeBuffer("pos", GL_FLOAT, 0, 3);
    shader.setUniformValue("mvp", projection * view * model);

    gl.glDrawElements(GL_LINES, 800, GL_UNSIGNED_INT, 0);

    shader.disableAttributeArray("pos");
    shader.release();
    indexBuffer->release();
    vertexBuffer->release();
}

bool PointCloudDrawable::init()
{
    shader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/pointcloud.glsl");
    shader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/pointcloud_fragment.glsl");
    shader.link();

    vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    vertexBuffer->create();
    vertexBuffer->bind();
    vertexBuffer->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vertexBuffer->release();

    return true;
}

void PointCloudDrawable::draw(QOpenGLFunctions& gl, const QMatrix4x4& projection, const QMatrix4x4& view)
{
    QMatrix4x4 model;
    model.setToIdentity();
    vertexBuffer->bind();
    shader.bind();
    shader.enableAttributeArray("pos");

    shader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, 2 * sizeof(glm::vec3));

    shader.enableAttributeArray("color");
    shader.setAttributeBuffer("color", GL_FLOAT, sizeof(glm::vec3), 3, 2 * sizeof(glm::vec3));
    shader.setUniformValue("mvp", projection * view * model);

    gl.glDrawArrays(GL_POINTS, 0, pointsSize);

    shader.disableAttributeArray("pos");
    shader.disableAttributeArray("color");

    shader.release();
    vertexBuffer->release();
}

void PointCloudDrawable::setPoints(const std::vector<glm::vec3>& newPoints)
{
    const auto requiredSize = static_cast<int>(sizeof(newPoints[0]) * newPoints.size());

    vertexBuffer->bind();
    if(vertexBuffer->size() < requiredSize)
        vertexBuffer->allocate(requiredSize);

    vertexBuffer->write(0, newPoints.data(), requiredSize);
    pointsSize = static_cast<int>(newPoints.size()) / 2;
    vertexBuffer->release();
}
