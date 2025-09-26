package com.example.agvtesterapp.websocket

import android.graphics.BitmapFactory
import android.util.Base64
import android.util.Log
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.models.CameraImage
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.models.Image
import com.example.agvtesterapp.util.ConnectionStatus
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken

class WebSocketDefaultEvents: WebSocketEvents {
    override fun onConnected(socketStatus: MutableLiveData<ConnectionStatus>) {
        socketStatus.postValue(ConnectionStatus.Connected())
    }

    override fun <T> onReceiveCameraImage(data: String, dataContainer: MutableLiveData<T>) {
        // Parsing JSON data into CameraImage object
//        val cameraImage = Gson().fromJson(data, CameraImage::class.java)
//
//        // Decoding Base64-encoded image to bytes (ByteArray)
//        val decodedBytes = Base64.decode(cameraImage.data, Base64.DEFAULT)
//
//        // Converting raw bytes to a Bitmap (for the UI)
//        val bitmap = BitmapFactory.decodeByteArray(decodedBytes, 0, decodedBytes.size)

        // ----------------------- Testing -----------------------------
        val cameraImage = Gson().fromJson(data, Image::class.java)
        val decodedBytes = Base64.decode(cameraImage.data, Base64.DEFAULT)
        val bitmap = BitmapFactory.decodeByteArray(decodedBytes, 0, decodedBytes.size)
        // ----------------------- Testing -----------------------------

        @Suppress("UNCHECKED_CAST")
        dataContainer.postValue(bitmap as T)

        Log.d(WebSocketClient.WEBSOCKET_TAG, "Image received")
    }

    override fun <T> onReceiveDetectedObject(data: String, dataContainer: MutableLiveData<T>) {

        val mapType = object : TypeToken<Map<String, Any>>() {}.type
        val detectedObjects: Map<String, Any> = Gson().fromJson(data, mapType)
        val list = (dataContainer.value as? MutableList<DetectedObject>) ?: mutableListOf() // Add safe cast

        for ((name, count) in detectedObjects) {
            val countInt = (count as? Int)?.toInt() ?: 0
            val existingObj = list.find {it.name == name}

            if (existingObj != null) {
                if (countInt > existingObj.count)
                   existingObj.count = countInt
            }
            else {
                list.add(DetectedObject(name = name, count = countInt))
            }
        }

        @Suppress("UNCHECKED_CAST")
        dataContainer.postValue(list as T)
    }

    override fun onDisconnected(socketStatus: MutableLiveData<ConnectionStatus>, reason: String?) {
        socketStatus.postValue(ConnectionStatus.Disconnected(reason))
    }
}


