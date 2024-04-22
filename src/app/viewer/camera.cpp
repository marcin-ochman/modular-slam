#include "camera.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

Camera::Camera(float fovy, float aspectRatio, float nearClip, float farClip)
    : m_fov(fovy), m_aspect(aspectRatio), m_near(nearClip), m_far(farClip),
      m_projection(glm::perspective(glm::radians(fovy), aspectRatio, nearClip, farClip))
{
    m_position = calculatePosition();
    updateView();
    updateProjection();
}

void Camera::setDistance(float distance) noexcept
{
    m_distance = distance;
    updateView();
}

void Camera::setViewportSize(float width, float height)
{
    m_viewportSize.x = width;
    m_viewportSize.y = height;
    updateProjection();
}

void Camera::setOrientation(glm::quat orientation)
{
    orientation = glm::normalize(orientation);
    m_pitch = glm::pitch(orientation);
    m_yaw = glm::yaw(orientation) + glm::pi<float>();
    m_roll = glm::roll(orientation);

    updateView();
}

void Camera::setPosition(glm::vec3 position)
{
    m_focalPoint = position + glm::vec3(0.0f, 0.0f, -1.0f) * m_distance;
}

glm::mat4 Camera::viewMatrix() const noexcept
{
    return m_view;
}

glm::mat4 Camera::projectionMatrix() const noexcept
{
    return m_projection;
}

glm::mat4 Camera::viewProjection() const noexcept
{
    return m_projection * m_view;
}

float Camera::distance() const noexcept
{
    return m_distance;
}

const glm::vec3& Camera::position() const noexcept
{
    return m_position;
}

float Camera::pitch() const noexcept
{
    return m_pitch;
}

float Camera::yaw() const noexcept
{
    return m_yaw;
}

void Camera::updateProjection()
{
    m_aspect = m_viewportSize.x / m_viewportSize.y;
    m_projection = glm::perspective(glm::radians(m_fov), m_aspect, m_near, m_far);
}

void Camera::updateView()
{
    m_position = calculatePosition();

    const auto orientation = this->orientation();
    m_view = glm::inverse(glm::translate(glm::mat4(1.0f), m_position) * glm::toMat4(orientation));
}

void Camera::resetView()
{
    m_distance = 2.0f;
    m_pitch = 3.141592f;
    m_yaw = 0.f;
    m_roll = 0.0f;

    updateView();
}

glm::vec2 Camera::panSpeed() const
{
    float x = std::min(m_viewportSize.x / 1000.0f, 2.5f);
    float xFactor = 0.04f * (x * x) - 0.18f * x + 0.30f;

    float y = std::min(m_viewportSize.y / 1000.0f, 2.5f);
    float yFactor = 0.04f * (y * y) - 0.18f * y + 0.30f;

    return glm::vec2(xFactor, yFactor);
}

float Camera::zoomSpeed() const
{
    float distance = m_distance * 0.2f;
    distance = std::max(distance, 0.0f);
    float speed = distance * distance;

    return std::min(speed, 100.0f);
}

void Camera::move_forward(float diff)
{
    constexpr auto forwardSpeed = 1.f;

    m_focalPoint += forward() * diff * forwardSpeed;

    updateView();
}

void Camera::pan(glm::vec2 diff)
{
    const auto speed = panSpeed();
    m_focalPoint += right() * diff.x * speed.x * m_distance;
    m_focalPoint += up() * diff.y * speed.y * m_distance;

    updateView();
}

void Camera::rotate(glm::vec2 move)
{
    constexpr auto rotationSpeed = 0.01f;

    float yawSign = -up().y < 0 ? -1.0f : 1.0f;
    m_yaw += yawSign * move.x * rotationSpeed;
    m_pitch += move.y * rotationSpeed;

    updateView();
}

void Camera::zoom(float delta)
{
    m_distance = std::max(m_distance - delta * zoomSpeed(), 0.2f);
    updateView();
}

glm::vec3 Camera::up() const noexcept
{
    return glm::rotate(orientation(), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::vec3 Camera::right() const noexcept
{
    return glm::rotate(orientation(), glm::vec3(-1.0f, 0.0f, 0.0f));
}

glm::vec3 Camera::forward() const noexcept
{
    return glm::rotate(orientation(), glm::vec3(0.0f, 0.0f, -1.0f));
}

glm::vec3 Camera::calculatePosition() const noexcept
{
    return m_focalPoint - forward() * m_distance;
}

glm::quat Camera::orientation() const noexcept
{
    return glm::quat(glm::vec3(-m_pitch, m_yaw, m_roll));
}
