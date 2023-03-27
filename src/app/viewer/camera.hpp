#ifndef MSLAM_VIEWER_CAMERA_HPP_
#define MSLAM_VIEWER_CAMERA_HPP_

#include <glm/glm.hpp>
#include <glm/matrix.hpp>

class Camera
{
  public:
    Camera(float fov, float aspectRatio, float nearClip, float farClip);

    void setDistance(float distance) noexcept;
    void setViewportSize(float width, float height);
    void setOrientation(glm::quat orientation);
    void setPosition(glm::vec3 position);

    glm::mat4 viewMatrix() const noexcept;
    glm::mat4 projectionMatrix() const noexcept;
    glm::mat4 viewProjection() const noexcept;

    float distance() const noexcept;
    const glm::vec3& position() const noexcept;
    float pitch() const noexcept;
    float yaw() const noexcept;
    glm::vec3 forward() const noexcept;
    glm::quat orientation() const noexcept;

    glm::vec3 up() const noexcept;
    glm::vec3 right() const noexcept;

    void pan(glm::vec2 diff);
    void rotate(glm::vec2 diff);
    void zoom(float delta);
    void updateView();

    void resetView();

  private:
    void updateProjection();
    glm::vec3 calculatePosition() const noexcept;
    glm::vec2 panSpeed() const;
    float zoomSpeed() const;

  private:
    float m_fov = 65.0f;
    float m_aspect = 1.778f;

    float m_near = 0.1f;
    float m_far = 1000.0f;

    glm::mat4 m_view{1.0};
    glm::mat4 m_projection{1.0};
    glm::vec3 m_position = {0.0f, 0.0f, 0.0f};
    glm::vec3 m_focalPoint = {0.0f, 0.0f, 0.0f};

    float m_distance = 2.0f;
    float m_pitch = 3.141592f;
    float m_yaw = 0.f;
    float m_roll = 0.0f;

    glm::vec2 m_viewportSize = {1280, 720};
};

#endif // MSLAM_VIEWER_CAMERA_HPP_
