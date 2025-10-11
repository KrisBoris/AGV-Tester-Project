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

class ViewModel(app: Application, val repository: Repository): AndroidViewModel(app) {

    var socketsStatus: Map<SocketType, MutableLiveData<ConnectionStatus>>

    val detectedObjects: MutableLiveData<MutableList<DetectedObject>> = MutableLiveData()
    val cameraImage: MutableLiveData<Bitmap> = MutableLiveData()

    // Shared preferences
    private val shared_prefs = "shared_prefs"
    private val ip_address_key = "ipaddr_key"

    private var ipAddress: String? = null

    init {
        ipAddress = getSharedPrefsString(shared_prefs, ip_address_key) ?: WebSocketClient.IP_ADDRESS

        socketsStatus = mapOf(
            SocketType.DETECTED_OBJECTS to MutableLiveData(),
            SocketType.CAMERA_IMAGE to MutableLiveData(),
            SocketType.STEERING to MutableLiveData())

        for(socket in SocketType.entries)
            setConnectionStatusReceiver(socket, socketsStatus[socket]!!)

        detectedObjects.value = mutableListOf()
    }

    private fun getSharedPrefsString(sharedPrefsKey: String, valueKey: String): String? =
        repository.getSharedPrefsString(sharedPrefsKey, valueKey)

    private fun putSharedPrefsString(sharedPrefsKey: String, valueKey: String, value: String) =
        repository.putSharedPrefsString(sharedPrefsKey, valueKey, value)

//    private val ports: Map<SocketType, String> = mapOf()

    fun getIpAddress(): String? = ipAddress

    fun setIpAddress(address: String?): Boolean {
        return if (address != null && InetAddressValidator.getInstance().isValidInet4Address(address)) {
            putSharedPrefsString(shared_prefs, ip_address_key, address)
            ipAddress = "ws://${address}"
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