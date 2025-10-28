package com.example.agvtesterapp.ui

import android.app.Application
import android.graphics.Bitmap
import android.util.Log
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

class ViewModel(app: Application, val repository: Repository): AndroidViewModel(app) {

    var socketsStatus: Map<SocketType, MutableLiveData<ConnectionStatus>>

    val detectedObjects: MutableLiveData<MutableList<DetectedObject>> = MutableLiveData()
    val cameraImage: MutableLiveData<Bitmap> = MutableLiveData()

    // Shared preferences
    private val shared_prefs = "shared_prefs"
    private val ip_address_key = "ipaddr_key"
    private val camera_socket_port_key = "camport_key"
    private val objects_socket_port_key = "objectsport_key"
    private val steering_socket_port_key = "steeringport_key"

    private var ipAddress: String? = null
    private val socketsPorts: MutableMap<SocketType, String> = mutableMapOf()

    init {
        Log.i("DEBUG_TAG", "ViewModel init")

        ipAddress = getSharedPrefsString(ip_address_key) ?: WebSocketClient.IP_ADDRESS

        socketsPorts[SocketType.CAMERA_IMAGE] = getSharedPrefsString(camera_socket_port_key)
            ?: WebSocketClient.CAMERA_SOCKET_PORT
        socketsPorts[SocketType.DETECTED_OBJECTS] = getSharedPrefsString(objects_socket_port_key)
            ?: WebSocketClient.OBJECTS_SOCKET_PORT
        socketsPorts[SocketType.STEERING] = getSharedPrefsString(steering_socket_port_key)
            ?: WebSocketClient.STEERING_SOCKET_PORT

        socketsStatus = mapOf(
            SocketType.DETECTED_OBJECTS to MutableLiveData(),
            SocketType.CAMERA_IMAGE to MutableLiveData(),
            SocketType.STEERING to MutableLiveData())

        for (socket in SocketType.entries) {
            val url = "ws://${ipAddress}:${socketsPorts[socket]}"
            setSocketIpAddress(socket, url)
            setConnectionStatusReceiver(socket, socketsStatus[socket]!!)
        }

        detectedObjects.value = mutableListOf()

        Log.i("DEBUG_TAG", "ViewModel end of init")
    }

    private fun getSharedPrefsString(valueKey: String): String? =
        repository.getSharedPrefsString(shared_prefs, valueKey)

    private fun putSharedPrefsString(valueKey: String, value: String) =
        repository.putSharedPrefsString(shared_prefs, valueKey, value)

//    private val ports: Map<SocketType, String> = mapOf()

    fun getIpAddress(): String? = ipAddress

    fun getSocketPort(socket: SocketType): String? = socketsPorts[socket]

    fun setIpAddress(address: String?): Boolean {
        return if (address != null && InetAddressValidator.getInstance().isValidInet4Address(address)) {
            putSharedPrefsString(ip_address_key, address)
            ipAddress = "ws://${address}"
            true
        } else
            false
    }

    fun setSocketPort(socket: SocketType, port: String): Boolean {
        val portNumber = port.toIntOrNull() ?: return false
        val isPortValid = portNumber in 1..65535
        return if (isPortValid) {
            when (socket) {
                SocketType.CAMERA_IMAGE -> putSharedPrefsString(camera_socket_port_key, port)
                SocketType.DETECTED_OBJECTS -> putSharedPrefsString(objects_socket_port_key, port)
                SocketType.STEERING -> putSharedPrefsString(steering_socket_port_key, port)
            }
            true
        } else
            false
    }

    fun addDetectedObject(detectedObject: DetectedObject) = viewModelScope.launch {
        repository.upsertResult(detectedObject)
    }

    fun getResults() = repository.getResults()

    fun clearResults() = viewModelScope.launch {
        repository.clearResults()
    }

    fun setSocketIpAddress(socket: SocketType, address: String) = repository.setSocketIpAddress(socket, address)
    fun setConnectionStatusReceiver(socket: SocketType, receiver: MutableLiveData<ConnectionStatus>) =
        repository.setConnectionStatusReceiver(socket, receiver)
    fun <T> connectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = viewModelScope.launch {
        repository.connectSocket(socket, dataReceiver)
    }
    fun <T> reconnectSocket(socket: SocketType, dataReceiver: MutableLiveData<T>? = null) = viewModelScope.launch {
        repository.reconnectSocket(socket, dataReceiver)
    }
    fun disconnectSocket(socket: SocketType) = viewModelScope.launch {
        repository.disconnectSocket(socket)
    }
    fun disconnectAllSockets() = viewModelScope.launch {
        for(socket in SocketType.entries)
            repository.disconnectSocket(socket)
    }
    fun <T> setDataReceiver(socket: SocketType, receiver: MutableLiveData<T>) = viewModelScope.launch {
        repository.setDataReceiver(socket, receiver)
    }
    fun sendSteeringCommand(socket: SocketType, cmd: Twist) = viewModelScope.launch {
        repository.sendSteeringCommand(socket, cmd)
    }
}