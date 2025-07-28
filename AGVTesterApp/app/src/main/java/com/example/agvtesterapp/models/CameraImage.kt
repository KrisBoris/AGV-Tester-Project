package com.example.agvtesterapp.models

import java.io.Serializable

data class CameraImage(
    val format: String,
    val image: ByteArray
): Serializable
