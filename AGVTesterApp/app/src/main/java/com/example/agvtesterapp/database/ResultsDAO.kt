package com.example.agvtesterapp.database

import androidx.lifecycle.LiveData
import androidx.room.Dao
import androidx.room.Insert
import androidx.room.OnConflictStrategy
import androidx.room.Query
import com.example.agvtesterapp.models.DetectedObject

/**
 * Dao interface that defines methods to access and manage database
 */
@Dao
interface ResultsDAO {

    /**
     * Inserts detected object into the database
     * If a conflict occurs (e.g.attempting to
     * insert a row with a primary key that already exists in the table)
     * than the existing row in the database will be replaced by the new row.
     * @param detectedObject detected object to insert
     */
    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun insert(detectedObject: DetectedObject): Long

    /**
     * @return list of all detected objects
     */
    @Query("SELECT * FROM detected_objects")
    fun getResults(): LiveData<List<DetectedObject>>

    /**
     * Removes all detected objects from the database
     */
    @Query("DELETE FROM detected_objects")
    suspend fun clearResults()
}