package com.example.agvtesterapp.models

/**
 * Data class defining structure of the image received from the AGV's camera
 */
data class CameraImage(
    val data: String    // Base64 string
)
