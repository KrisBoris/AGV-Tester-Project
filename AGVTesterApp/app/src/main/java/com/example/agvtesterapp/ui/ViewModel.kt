package com.example.agvtesterapp.ui

import android.app.Application
import android.graphics.Bitmap
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.viewModelScope
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.repository.Repository
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import kotlinx.coroutines.launch

class ViewModel(app: Application, val repository: Repository): AndroidViewModel(app) {

    var socketsStatus: Map<SocketType, MutableLiveData<ConnectionStatus>>

    val detectedObjects: MutableLiveData<MutableList<DetectedObject>> = MutableLiveData()
    val cameraImage: MutableLiveData<Bitmap> = MutableLiveData()

    // Shared preferences
    private val shared_prefs = "shared_prefs"
    private val ip_address_key = "ipaddr_key"

    init {
        setIpAddress(getSharedPrefsString(shared_prefs, ip_address_key))

        socketsStatus = mapOf(
            SocketType.DETECTED_OBJECTS to MutableLiveData(),
            SocketType.CAMERA_IMAGE to MutableLiveData(),
            SocketType.STEERING to MutableLiveData())

        for(socket in SocketType.entries)
            setConnectionStatusReceiver(socket, socketsStatus[socket]!!)
    }

    fun getSharedPrefsKey() = shared_prefs
    fun getIpAddrKey() = ip_address_key

    private var ipAddress: String? = null

    fun getIpAddress(): String? = ipAddress
    fun setIpAddress(address: String?) {
        ipAddress = address
    }

    fun getSharedPrefsString(sharedPrefsKey: String, valueKey: String): String? =
        repository.getSharedPrefsString(sharedPrefsKey, valueKey)

    fun putSharedPrefsString(sharedPrefsKey: String, valueKey: String, value: String) =
        repository.putSharedPrefsString(sharedPrefsKey, valueKey, value)

    fun addDetectedObject(detectedObject: DetectedObject) = viewModelScope.launch {
        repository.upsertResult(detectedObject)
    }

    fun getResults() = repository.getResults()

    fun clearResults() = viewModelScope.launch {
        repository.clearResults()
    }

    fun setConnectionStatusReceiver(socket: SocketType, receiver: MutableLiveData<ConnectionStatus>) =
        repository.setConnectionStatusReceiver(socket, receiver)
    fun connectSocket(socket: SocketType) = viewModelScope.launch {
        repository.connectSocket(socket)
    }
    fun reconnectSocket(socket: SocketType) = viewModelScope.launch {
        repository.reconnectSocket(socket)
    }
    fun disconnectSocket(socket: SocketType) = viewModelScope.launch {
        repository.disconnectSocket(socket)
    }
    fun <T> setDataReceiver(socket: SocketType, receiver: MutableLiveData<T>) = viewModelScope.launch {
        repository.setDataReceiver(socket, receiver)
    }
    fun sendSteeringCommand(socket: SocketType, message: String) = viewModelScope.launch {
        repository.sendSteeringCommand(socket, message)
    }
}