#ifndef MSLAM_VIEWER_CAMERA_HPP_
#define MSLAM_VIEWER_CAMERA_HPP_

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glm/matrix.hpp>
#include <vector>

#include <iostream>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific
// input methods
enum CameraMovement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 75.0f;

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for
// use in OpenGL
class Camera
{
  public:
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    float Yaw;
    float Pitch;
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           float yaw = YAW, float pitch = PITCH)
        : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
    {
        Position = position;
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        updateCameraVectors();
    }

    glm::mat4 getViewMatrix()
    {

        auto v = glm::transpose(glm::lookAt(Position, Position + Front, Up));
        std::cout << v[0].x << " " << v[0].y << " " << v[0].z << " " << v[0].w << std::endl
                  << v[1].x << " " << v[1].y << " " << v[1].z << " " << v[1].w << std::endl
                  << v[2].x << " " << v[2].y << " " << v[2].z << " " << v[2].w << std::endl
                  << v[3].x << " " << v[3].y << " " << v[3].z << " " << v[3].w << std::endl
                  << std::endl;

        return glm::lookAt(Position, Position + Front, Up);
    }

    // void processKeyboard(CameraMovement direction, float deltaTime)
    // {
    //     float velocity = MovementSpeed * deltaTime;
    //     if(direction == FORWARD)
    //         Position += Front * velocity;
    //     if(direction == BACKWARD)
    //         Position -= Front * velocity;
    //     if(direction == LEFT)
    //         Position -= Right * velocity;
    //     if(direction == RIGHT)
    //         Position += Right * velocity;
    // }
    //
    void processCameraPlaneMovement(float xoffset, float yoffset)
    {
        Position -= Right * xoffset;
        Position += Up * yoffset;
    }

    void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw += xoffset;
        Pitch += yoffset;

        if(constrainPitch)
        {
            if(Pitch > 89.0f)
                Pitch = 89.0f;
            if(Pitch < -89.0f)
                Pitch = -89.0f;
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
        // calculate the new Front vector
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Pitch));
        front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = glm::normalize(front);
        // also re-calculate the Right and Up vector
        Right = glm::normalize(
            glm::cross(Front, WorldUp)); // normalize the vectors, because their length gets closer to 0 the more you
                                         // look up or down which results in slower movement.
        Up = glm::normalize(glm::cross(Right, Front));
    }
};

#endif // MSLAM_VIEWER_CAMERA_HPP_
