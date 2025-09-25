package com.example.agvtesterapp.repository

import android.content.Context
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.database.ResultsDatabase
import com.example.agvtesterapp.models.DetectedObjects
import com.example.agvtesterapp.models.Twist
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient

class Repository(
    private val context: Context,
    private val db: ResultsDatabase,
    private val sockets: Map<SocketType, WebSocketClient>
) {
    fun getSharedPrefsString(sharedPrefsKey: String, valueKey: String): String? {
        val sharedPreferences = context.getSharedPreferences(sharedPrefsKey, Context.MODE_PRIVATE)

        return sharedPreferences.getString(valueKey, null)
    }

    fun putSharedPrefsString(sharedPrefsKey: String, valueKey: String, value: String) {
        val sharedPreferences = context.getSharedPreferences(sharedPrefsKey, Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()

        editor.putString(valueKey, value)
        editor.apply()
    }
    suspend fun upsertResult(detectedObjects: DetectedObjects) = db.getResultsDAO().upsert(detectedObjects)
    fun getResults() = db.getResultsDAO().getResults()
    suspend fun clearResults() = db.getResultsDAO().clearResults()

    fun setConnectionStatusReceiver(socket: SocketType, receiver: MutableLiveData<ConnectionStatus>) =
        sockets[socket]?.setConnectionStatusReceiver(receiver)
    suspend fun <T> connectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = sockets[socket]?.connect(socket, dataReceiver)
    suspend fun <T> reconnectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = sockets[socket]?.reconnect(socket, dataReceiver)
    suspend fun disconnectSocket(socket: SocketType) = sockets[socket]?.disconnect()
    suspend fun <T> setDataReceiver(socket: SocketType, receiver: MutableLiveData<T>) =
        sockets[socket]?.setDataReceiver(receiver)
    suspend fun sendSteeringCommand(socket: SocketType, cmd: Twist) = sockets[socket]?.send(cmd)
}