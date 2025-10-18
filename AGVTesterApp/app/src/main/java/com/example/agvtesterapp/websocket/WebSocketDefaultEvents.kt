package com.example.agvtesterapp.websocket

import android.graphics.BitmapFactory
import android.util.Base64
import android.util.Log
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.models.CameraImage
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.util.ConnectionStatus
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken

/**
 * Class with definitions of methods called upon designated WebSocket client's events
 * (e.g. connecting to a server or receiving certain type of message from a server)
 */
class WebSocketDefaultEvents: WebSocketEvents {

    /**
     * Method sets socket's status to connected after successful connection
     * @param socketStatus MutableLiveData object holding current connection status
     */
    override fun onConnected(socketStatus: MutableLiveData<ConnectionStatus>) {
        socketStatus.postValue(ConnectionStatus.Connected())
    }

    /**
     * Method sets socket's status to disconnected after losing the connection
     * @param socketStatus MutableLiveData object holding current connection status
     */
    override fun onDisconnected(socketStatus: MutableLiveData<ConnectionStatus>, reason: String?) {
        socketStatus.postValue(ConnectionStatus.Disconnected(reason))
    }

    /**
     * Method receives image in JSON format, converts it to bitmap and puts it in given container
     * @param data image in JSON format
     * @param dataContainer container where the image will be stored (in bitmap format)
     */
    override fun <T> onReceiveCameraImage(data: String, dataContainer: MutableLiveData<T>) {
        // Parsing JSON data into CameraImage object
        val cameraImage = Gson().fromJson(data, CameraImage::class.java)

        // Decoding Base64-encoded image to bytes (ByteArray)
        val decodedBytes = Base64.decode(cameraImage.data, Base64.DEFAULT)

        // Converting raw bytes to a Bitmap (for the UI)
        val bitmap = BitmapFactory.decodeByteArray(decodedBytes, 0, decodedBytes.size)

        // If received image was damaged the BitmapFactory will return null
        if (bitmap != null) {
            @Suppress("UNCHECKED_CAST")
            dataContainer.postValue(bitmap as T)

            Log.d(WebSocketClient.WEBSOCKET_TAG, "Image received")
        }
        else
            Log.e(WebSocketClient.WEBSOCKET_TAG, "Failed to decode image: ${decodedBytes.size} bytes")

    }

    /**
     * Method receives the list of detected objects with their respective counts and updates
     * current list of detected objects based on that
     * @param data list of detected objects with their respective counts in JSON format
     * @param dataContainer container where list of detected objects will be stored
     */
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
}