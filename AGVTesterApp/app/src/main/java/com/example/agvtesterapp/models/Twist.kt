package com.example.agvtesterapp.models

/**
 * Data class defining structure of the message type used steer the AGV
 * It corresponds to ROS geometry_msgs/Twist message type
 */
data class Twist(
    val linear: Vector3,
    val angular: Vector3
)
