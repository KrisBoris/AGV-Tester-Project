package com.example.agvtesterapp.websocket

import android.graphics.BitmapFactory
import android.util.Base64
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.util.ConnectionStatus

class WebSocketDefaultEvents: WebSocketEvents {
    override fun onConnected(socketStatus: MutableLiveData<ConnectionStatus>) {
        socketStatus.postValue(ConnectionStatus.Connected())
    }

    override fun <T> onReceiveCameraImage(data: String, dataContainer: MutableLiveData<T>) {
        val decodedBytes = Base64.decode(data, Base64.DEFAULT)
        val bitmap = BitmapFactory.decodeByteArray(decodedBytes, 0, decodedBytes.size)

        @Suppress("UNCHECKED_CAST")
        dataContainer.postValue(bitmap as T)
    }

    override fun <T> onReceiveDetectedObject(data: String, dataContainer: MutableLiveData<T>) {
        // add converter later

        val detectedObject = DetectedObject(0, "k", "k")

        val list = dataContainer.value as MutableList<Any>
        list.add(detectedObject)

        @Suppress("UNCHECKED_CAST")
        dataContainer.postValue(list as T)
    }

    override fun onDisconnected(socketStatus: MutableLiveData<ConnectionStatus>, reason: String?) {
        socketStatus.postValue(ConnectionStatus.Disconnected(reason))
    }
}


