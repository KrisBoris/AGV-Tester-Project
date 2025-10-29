package com.example.agvtesterapp.util

/**
 * Enum class containing all types of sockets used by application
 */
enum class SocketType {
    /**
     * Socket for receiving camera image
     */
    CAMERA_IMAGE,

    /**
     * Socket for receiving detected objects
     */
    DETECTED_OBJECTS,

    /**
     * Socket for sending steering commands
     */
    STEERING
}