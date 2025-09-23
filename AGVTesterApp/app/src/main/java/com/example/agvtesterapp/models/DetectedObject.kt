package com.example.agvtesterapp.models

import java.io.Serializable

data class DetectedObject(
    val name: String,
    var count: Int
): Serializable