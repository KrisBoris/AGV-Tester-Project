package com.example.agvtesterapp.ui

import android.app.Application
import android.graphics.Bitmap
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.viewModelScope
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.models.Twist
import com.example.agvtesterapp.repository.Repository
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient
import kotlinx.coroutines.launch
import org.apache.commons.validator.routines.InetAddressValidator

/**
 * View model class - used to share data and methods between activities and fragments
 * @param app reference to the [Application] class that owns this view model
 * @param repository reference to the [Repository] instance
 */
class ViewModel(app: Application, val repository: Repository): AndroidViewModel(app) {

    /**
     * Shared preferences file key
     */
    private val shared_prefs = "shared_prefs"

    /**
     * IP address key for shared preferences
     */
    private val ip_address_key = "ipaddr_key"

    /**
     * Camera WebSocket port key for shared preferences
     */
    private val camera_socket_port_key = "camport_key"

    /**
     * Detected objects WebSocket port key for shared preferences
     */
    private val objects_socket_port_key = "objectsport_key"

    /**
     * Steering WebSocket port key for shared preferences
     */
    private val steering_socket_port_key = "steeringport_key"

    /**
     * WebSocket server IP address
     */
    private var ipAddress: String? = null

    /**
     * WebSocket server socket types mapped to respective port numbers
     */
    private val socketsPorts: MutableMap<SocketType, String> = mutableMapOf()


    /**
     * WebSocket server socket types mapped to respective containers holding their connection status
     */
    var socketsStatus: Map<SocketType, MutableLiveData<ConnectionStatus>>

    /**
     * Container storing detected objects received from WebSocket server
     */
    val detectedObjects: MutableLiveData<MutableList<DetectedObject>> = MutableLiveData()

    /**
     * Container storing image in form of [Bitmap] received from WebSocket server
     * sending image from AGV's camera
     */
    val cameraImage: MutableLiveData<Bitmap> = MutableLiveData()


    init {
        // Get IP address and ports values from shared preferences
        // or from WebSocketClient companion object if absent
        ipAddress = getSharedPrefsString(ip_address_key) ?: WebSocketClient.IP_ADDRESS

        socketsPorts[SocketType.CAMERA_IMAGE] = getSharedPrefsString(camera_socket_port_key)
            ?: WebSocketClient.CAMERA_SOCKET_PORT
        socketsPorts[SocketType.DETECTED_OBJECTS] = getSharedPrefsString(objects_socket_port_key)
            ?: WebSocketClient.OBJECTS_SOCKET_PORT
        socketsPorts[SocketType.STEERING] = getSharedPrefsString(steering_socket_port_key)
            ?: WebSocketClient.STEERING_SOCKET_PORT

        // Init with empty MutableLiveData objects
        socketsStatus = mapOf(
            SocketType.DETECTED_OBJECTS to MutableLiveData(),
            SocketType.CAMERA_IMAGE to MutableLiveData(),
            SocketType.STEERING to MutableLiveData())

        // Set each WebSocket client's IP address and container for connection status
        for (socket in SocketType.entries) {
            val url = "ws://${ipAddress}:${socketsPorts[socket]}" // "ws" prefix to inform that it is WebSocket server
            setSocketIpAddress(socket, url)
            setConnectionStatusReceiver(socket, socketsStatus[socket]!!)
        }

        // Init with empty MutableList object
        detectedObjects.value = mutableListOf()
    }


    /**
     * Gets [String] value from application's shared preferences file
     * @param key desired value's key
     * @return [String] value paired with given [key] or null if not found
     */
    private fun getSharedPrefsString(key: String): String? =
        repository.getSharedPrefsString(shared_prefs, key)

    /**
     * Saves given key-value pair to application's shared preferences file
     * @param key value key
     * @param value [String] value
     */
    private fun putSharedPrefsString(key: String, value: String) =
        repository.putSharedPrefsString(shared_prefs, key, value)


    /**
     * @return WebSocket server IP address
     */
    fun getIpAddress(): String? = ipAddress

    /**
     * @param socket socket type
     * @return WebSocket server socket port for specified socket type or null if not found
     */
    fun getSocketPort(socket: SocketType): String? = socketsPorts[socket]

    /**
     * Validates given IP address value. If it is correct, saves it in shared preferences and
     * sets IP address value stored in view model to the new one
     * @param address WebSocket server IP address (IPv4 protocol)
     * @return true if IP address is valid, false otherwise
     */
    fun setIpAddress(address: String?): Boolean {
        return if (address != null && InetAddressValidator.getInstance().isValidInet4Address(address)) {
            putSharedPrefsString(ip_address_key, address)
            ipAddress = "ws://${address}"
            true
        } else
            false
    }

    /**
     * Validates given WebSocket socket port number. If it is correct saves it in shared preferences
     * and sets designated socket port value to the new one
     * @param socket socket type
     * @param port port number
     * @return true if port number is valid, false otherwise
     */
    fun setSocketPort(socket: SocketType, port: String): Boolean {
        // Parse port number to Int - return false in case of failure
        val portNumber = port.toIntOrNull() ?: return false

        // Check whether the port number is in correct range (from 1 inclusive to 65535 inclusive)
        val isPortValid = portNumber in 1..65535

        return if (isPortValid) {
            when (socket) {
                SocketType.CAMERA_IMAGE -> {
                    socketsPorts[SocketType.CAMERA_IMAGE] = port
                    putSharedPrefsString(camera_socket_port_key, port)
                }
                SocketType.DETECTED_OBJECTS -> {
                    socketsPorts[SocketType.DETECTED_OBJECTS] = port
                    putSharedPrefsString(objects_socket_port_key, port)
                }
                SocketType.STEERING -> {
                    socketsPorts[SocketType.STEERING] = port
                    putSharedPrefsString(steering_socket_port_key, port)
                }
            }
            true
        } else
            false
    }

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and adds detected object to the database
     * using repository's method (asynchronously, non-blocking)
     * @param detectedObject detected object that will by insert into database
     * @return handle to the running coroutine
     */
    fun addDetectedObject(detectedObject: DetectedObject) = viewModelScope.launch {
        repository.insertResult(detectedObject)
    }

    /**
     * Returns the list of all detected objects from database using repository's method
     * @return list of all detected objects
     */
    fun getResults() = repository.getResults()

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and removes all detected objects
     * from the database using repository's method (asynchronously, non-blocking)
     * @return handle to the running coroutine
     */
    fun clearResults() = viewModelScope.launch {
        repository.clearResults()
    }

    /**
     * Sets up WebSocket client instance's IP address using repository's method
     * @param socket socket type
     * @param address IP address (IPv4 protocol)
     */
    fun setSocketIpAddress(socket: SocketType, address: String) = repository.setSocketIpAddress(socket, address)

    /**
     * Passes reference to the container holding connection status, to the WebSocket client instance
     * using repository's method
     * @param socket socket type
     * @param receiver reference to the container holding connection status
     */
    fun setConnectionStatusReceiver(socket: SocketType, receiver: MutableLiveData<ConnectionStatus>) =
        repository.setConnectionStatusReceiver(socket, receiver)

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and connects WebSocket client
     * to the WebSocket server specified by client's IP address using repository's method
     * (asynchronously, non-blocking)
     * @param socket socket type
     * @param dataReceiver reference to the container for data received from server
     * @return handle to the running coroutine
     */
    fun <T> connectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = viewModelScope.launch {
        repository.connectSocket(socket, dataReceiver)
    }

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and reconnects WebSocket client
     * to the WebSocket server specified by client's IP address using repository's method
     * (asynchronously, non-blocking)
     * @param socket socket type
     * @param dataReceiver reference to the container for data received from server
     * @return handle to the running coroutine
     */
    fun <T> reconnectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = viewModelScope.launch {
        repository.reconnectSocket(socket, dataReceiver)
    }

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and disconnects WebSocket client
     * to the WebSocket server specified by client's IP address using repository's method
     * (asynchronously, non-blocking)
     * @param socket socket type
     * @return handle to the running coroutine
     */
    fun disconnectSocket(socket: SocketType) = viewModelScope.launch {
        repository.disconnectSocket(socket)
    }

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and disconnects all WebSocket clients
     * from the WebSocket servers specified by clients' IP addresses using repository's method
     * (asynchronously, non-blocking)
     * @return handle to the running coroutine
     */
    fun disconnectAllSockets() = viewModelScope.launch {
        for(socket in SocketType.entries)
            repository.disconnectSocket(socket)
    }

    /**
     * Runs a coroutine inside ViewModel's [viewModelScope] and sends steering command
     * to the WebSocket server using repository's method (asynchronously, non-blocking)
     * @param cmd steering command corresponding to ROS geometry_msgs/Twist message type
     * @return handle to the running coroutine
     */
    fun sendSteeringCommand(cmd: Twist) = viewModelScope.launch {
        repository.sendSteeringCommand(cmd)
    }
}