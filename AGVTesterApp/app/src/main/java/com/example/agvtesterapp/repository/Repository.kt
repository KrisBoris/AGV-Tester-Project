package com.example.agvtesterapp.repository

import android.content.Context
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.database.ResultsDatabase
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.models.Twist
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient

/**
 * Class that provides methods to access all external objects and data sources
 * @param context application context used to access shared preferences
 * @param db [ResultsDatabase] instance that stores detected objects
 * @param sockets [WebSocketClient] instances mapped to dedicated [SocketType]
 */
class Repository(
    private val context: Context,
    private val db: ResultsDatabase,
    private val sockets: Map<SocketType, WebSocketClient>
) {

    /**
     * Gets [String] value for given key, from designated shared preferences file
     * @param sharedPrefsKey shared preferences file's key
     * @param key desired value's key
     * @return [String] value paired with given [key] or null if not found
     */
    fun getSharedPrefsString(sharedPrefsKey: String, key: String): String? {
        val sharedPreferences = context.getSharedPreferences(sharedPrefsKey, Context.MODE_PRIVATE)
        return sharedPreferences.getString(key, null)
    }

    /**
     * Saves given key-value pair to dedicated shared preferences file
     * @param sharedPrefsKey shared preferences file's key
     * @param key value's key
     * @param value [String] value
     */
    fun putSharedPrefsString(sharedPrefsKey: String, key: String, value: String) {
        val sharedPreferences = context.getSharedPreferences(sharedPrefsKey, Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString(key, value)
        editor.apply()
    }


    /**
     * Inserts given detected object into the database
     * @param detectedObject detected object that will by insert into database
     * @return the row ID of the inserted record in the table
     */
    suspend fun insertResult(detectedObject: DetectedObject) = db.getResultsDAO().insert(detectedObject)

    /**
     * Accesses database to retrieve list of all detected objects
     * @return list of all detected objects
     */
    fun getResults() = db.getResultsDAO().getResults()

    /**
     * Accesses database to removes all detected objects from it
     */
    suspend fun clearResults() = db.getResultsDAO().clearResults()


    /**
     * Sets up WebSocket client instance's IP address
     * @param socket socket type
     * @param address IP address (IPv4 protocol)
     */
    fun setSocketIpAddress(socket: SocketType, address: String) = sockets[socket]?.setIpAddress(address)

    /**
     * Passes reference to the container holding connection status, to the WebSocket client instance
     * @param socket socket type
     * @param receiver reference to the container holding connection status
     */
    fun setConnectionStatusReceiver(socket: SocketType, receiver: MutableLiveData<ConnectionStatus>) =
        sockets[socket]?.setConnectionStatusReceiver(receiver)

    /**
     * Connects WebSocket client to the WebSocket server specified by client's IP address
     * @param socket socket type
     * @param dataReceiver reference to the container for data received from server
     */
    suspend fun <T> connectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = sockets[socket]?.connect(socket, dataReceiver)

    /**
     * Reconnects WebSocket client to the WebSocket server specified by client's IP address
     * @param socket socket type
     * @param dataReceiver reference to the container for data received from server
     */
    suspend fun <T> reconnectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = sockets[socket]?.reconnect(socket, dataReceiver)

    /**
     * Disconnects WebSocket client from WebSocket server
     * @param socket socket type
     */
    suspend fun disconnectSocket(socket: SocketType) = sockets[socket]?.disconnect()

    /**
     * Sends steering command to the WebSocket server
     * @param cmd steering command corresponding to ROS geometry_msgs/Twist message type
     */
    suspend fun sendSteeringCommand(cmd: Twist) = sockets[SocketType.STEERING]?.send(cmd)
}