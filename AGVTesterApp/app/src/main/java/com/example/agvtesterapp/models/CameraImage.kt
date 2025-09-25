package com.example.agvtesterapp.models

import java.io.Serializable

data class CameraImage(
    val header: Header,
    val format: String,
    val data: String    // Base64 string
): Serializable
