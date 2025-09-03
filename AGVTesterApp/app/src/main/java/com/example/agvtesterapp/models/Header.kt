package com.example.agvtesterapp.models

import java.io.Serializable

data class Header(
    val seq: Int,
    val stamp: Long,
    val frameId: String
): Serializable
