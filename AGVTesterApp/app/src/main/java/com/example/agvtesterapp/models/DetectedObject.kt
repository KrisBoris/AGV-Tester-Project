package com.example.agvtesterapp.models

import androidx.room.Entity
import androidx.room.PrimaryKey
import java.io.Serializable

@Entity (tableName = "detected_objects")
data class DetectedObject(
    @PrimaryKey(autoGenerate = true)
    var id: Int? = null,
    val name: String,
    var count: Int
): Serializable
