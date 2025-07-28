package com.example.agvtesterapp.database

import androidx.lifecycle.LiveData
import androidx.room.Dao
import androidx.room.Delete
import androidx.room.Insert
import androidx.room.OnConflictStrategy
import androidx.room.Query
import com.example.agvtesterapp.models.DetectedObject

@Dao
interface ResultsDAO {

    // If a conflict occurs (e.g.attempting to
    // insert a row with a primary key that already exists in the table)
    // than the existing row in the database will be replaced by the new row.
    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun upsert(detectedObject: DetectedObject): Long

    @Query("SELECT * FROM detected_objects")
    fun getResults(): LiveData<List<DetectedObject>>

    @Query("DELETE FROM detected_objects")
    suspend fun clearResults()
}