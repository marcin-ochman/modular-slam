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

    for(int j = 0; j <= cellsCount; ++j)
    {
        for(int i = 0; i <= cellsCount; ++i)
        {
            float x = i - cellsCount / 2.0f;
            float z = -(j - cellsCount / 2.0f);
            float y = 0.0f;
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
    vertexBuffer->allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
    vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vertexBuffer->release();

    indexBuffer->create();
    indexBuffer->bind();
    indexBuffer->allocate(indices.data(), sizeof(indices[0]) * indices.size());
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
    model.rotate(180, 1, 0, 0);
    vertexBuffer->bind();
    shader.bind();
    shader.enableAttributeArray("pos");

    shader.setAttributeBuffer("pos", GL_FLOAT, 0, 3, 2 * sizeof(glm::vec3));

    shader.enableAttributeArray("color");
    shader.setAttributeBuffer("color", GL_FLOAT, sizeof(glm::vec3), 3, 2 * sizeof(glm::vec3));
    shader.setUniformValue("mvp", projection * view * model);

    gl.glDrawArrays(GL_POINTS, 0, vertexBuffer->size() / (2 * sizeof(glm::vec3)));

    shader.disableAttributeArray("pos");
    shader.disableAttributeArray("color");

    shader.release();
    vertexBuffer->release();
}

void PointCloudDrawable::setPoints(const std::vector<glm::vec3>& newPoints)
{
    vertexBuffer->bind();
    vertexBuffer->allocate(&newPoints[0], sizeof(newPoints[0]) * newPoints.size());
    vertexBuffer->release();
}
