package com.example.agvtesterapp.models

import java.io.Serializable

data class Twist(
    val linear: Vector3,
    val angular: Vector3
): Serializable
