package com.example.agvtesterapp.models

import androidx.room.Entity
import androidx.room.PrimaryKey
import java.io.Serializable

@Entity (tableName = "detected_objects")
data class DetectedObject(
    @PrimaryKey(autoGenerate = true)
    var id: Int,
    val name: String,
    val count: Int
): Serializable
