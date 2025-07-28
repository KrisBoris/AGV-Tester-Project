package com.example.agvtesterapp.websocket

import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.util.ConnectionStatus

interface WebSocketEvents {
    fun onConnected(socketStatus: MutableLiveData<ConnectionStatus>)
    fun <T> onReceiveCameraImage(data: String, dataContainer: MutableLiveData<T>)
    fun <T> onReceiveDetectedObject(data: String, dataContainer: MutableLiveData<T>)
    fun onDisconnected(socketStatus: MutableLiveData<ConnectionStatus>, reason: String? = null)
}