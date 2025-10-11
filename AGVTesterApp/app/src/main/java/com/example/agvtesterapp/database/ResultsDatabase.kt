package com.example.agvtesterapp.database

import android.content.Context
import androidx.room.Database
import androidx.room.Room
import androidx.room.RoomDatabase
import com.example.agvtesterapp.models.DetectedObject

@Database(
    entities = [DetectedObject::class],
    version = 1
)

abstract class ResultsDatabase: RoomDatabase() {

    abstract fun getResultsDAO(): ResultsDAO

    companion object {

        // Volatile - Prevents threads from using outdated references to the instance
        @Volatile
        private var instance: ResultsDatabase? = null

        // Used for synchronizing database creation to ensure that
        // only one thread can initialize the instance at a time.
        private val LOCK = Any()

        // Allows the class to be called like a function
        operator fun invoke(context: Context) = instance ?: synchronized(LOCK) {
            instance ?: createDatabase(context).also {
                instance = it
            }
        }

        private fun createDatabase(context: Context) =
            Room.databaseBuilder(
                context.applicationContext,
                ResultsDatabase::class.java,
                "results_db.db"
            ).build()
    }
}