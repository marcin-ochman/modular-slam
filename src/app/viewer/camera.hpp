

#ifndef MSLAM_VIEWER_CAMERA_HPP_
#define MSLAM_VIEWER_CAMERA_HPP_

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glm/matrix.hpp>
#include <vector>

// Default camera values
const float SENSITIVITY = 0.1f;
const float ZOOM = 75.0f;

class Camera
{
  public:
    float Zoom;

    Camera(glm::vec3 _position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           float _yaw = -90.0f, float _pitch = 0.0f)
        : Zoom(ZOOM), front(glm::vec3(0.0f, 0.0f, -1.0f)), mouseSensitivity(SENSITIVITY)
    {
        this->position = _position;
        worldUp = up;
        this->yaw = _yaw;
        this->pitch = _pitch;
        updateCameraVectors();
    }

    glm::mat4 getViewMatrix() { return glm::lookAt(position, position + front, up); }

    void processCameraPlaneMovement(float xoffset, float yoffset)
    {
        position -= right * xoffset;
        position += up * yoffset;
    }

    void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true)
    {
        xoffset *= mouseSensitivity;
        yoffset *= mouseSensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if(constrainPitch)
        {
            if(pitch > 89.0f)
                pitch = 89.0f;
            if(pitch < -89.0f)
                pitch = -89.0f;
        }

        updateCameraVectors();
    }

    void processMouseScroll(float yoffset)
    {
        Zoom -= (float)yoffset;
        if(Zoom < 1.0f)
            Zoom = 1.0f;
        if(Zoom > 85.0f)
            Zoom = 85.0f;
    }

  private:
    void updateCameraVectors()
    {
        front.x = std::cos(glm::radians(yaw)) * std::cos(glm::radians(pitch));
        front.y = std::sin(glm::radians(pitch));
        front.z = std::sin(glm::radians(yaw)) * std::cos(glm::radians(pitch));
        front = glm::normalize(front);

        right = glm::normalize(glm::cross(front, worldUp));
        up = glm::normalize(glm::cross(right, front));
    }

    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;
    float yaw;
    float pitch;
    float mouseSensitivity;
};

#endif // MSLAM_VIEWER_CAMERA_HPP_
