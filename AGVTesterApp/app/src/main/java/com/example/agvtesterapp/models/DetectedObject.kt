package com.example.agvtesterapp.models

import androidx.room.Entity
import androidx.room.PrimaryKey
import java.io.Serializable

/**
 * Data class defining structure of the database table storing detected objects
 */
@Entity (tableName = "detected_objects")
data class DetectedObject(
    @PrimaryKey(autoGenerate = true)
    var id: Int? = null,
    val name: String,   // name of the object's class
    var count: Int      // the amount of objects captured at once (in one frame)
): Serializable
