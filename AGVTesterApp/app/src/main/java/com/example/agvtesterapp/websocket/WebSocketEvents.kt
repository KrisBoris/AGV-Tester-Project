package com.example.agvtesterapp.websocket

import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.util.ConnectionStatus

/**
 * Interface with methods called upon designated WebSocket client's events
 */
interface WebSocketEvents {
    /**
     * Called upon WebSocket client establishing connection with WebSocket server
     * @param socketStatus MutableLiveData object holding current connection status
     */
    fun onConnected(socketStatus: MutableLiveData<ConnectionStatus>)

    /**
     * Called upon WebSocket client losing connection with WebSocket server
     * @param socketStatus MutableLiveData object holding current connection status
     */
    fun onDisconnected(socketStatus: MutableLiveData<ConnectionStatus>, reason: String? = null)

    /**
     * Called upon receiving message containing camera image from WebSocket server
     * @param data image in JSON format
     * @param dataContainer container where the image will be stored
     */
    fun <T> onReceiveCameraImage(data: String, dataContainer: MutableLiveData<T>)

    /**
     * Called upon receiving message containing list of detected objects from WebSocket server
     * @param data list of detected objects with their respective counts in JSON format
     * @param dataContainer container where list of detected objects will be stored
     */
    fun <T> onReceiveDetectedObject(data: String, dataContainer: MutableLiveData<T>)

}