package com.example.agvtesterapp.websocket

import android.graphics.Bitmap
import android.util.Log
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.util.ConnectionStatus
import io.ktor.client.HttpClient
import io.ktor.client.engine.okhttp.OkHttp
import io.ktor.client.plugins.logging.Logging
import io.ktor.client.plugins.websocket.WebSockets
import io.ktor.client.plugins.websocket.webSocketSession
import io.ktor.websocket.Frame
import io.ktor.websocket.WebSocketSession
import io.ktor.websocket.close
import io.ktor.websocket.readText
import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.filterIsInstance
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.plus
import java.util.concurrent.TimeUnit

 class WebSocketClient(
     private val url: String,
     private var socketStatus: MutableLiveData<ConnectionStatus>? = null
) {
    companion object {
        const val IP_ADDRESS = "ws://10.0.2.2:8080"
        const val WEBSOCKET_TAG = "WEBSOCKET_TAG"
        const val PING_INTERVAL = 40L   // 25 FPS
        const val RECONNECT_DELAY = 3_000L
        const val RECONNECTION_ATTEMPTS = 3
    }

    private val client = HttpClient(OkHttp) {
        engine {
            config {
                pingInterval(PING_INTERVAL, TimeUnit.MILLISECONDS)
            }
        }

        install(Logging)
        install(WebSockets)
    }

    private val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() +
            CoroutineExceptionHandler { _, throwable ->
                Log.d(WEBSOCKET_TAG, "Error: ${throwable.message}")
            }

    private var job: Job? = null

    private var session: WebSocketSession? = null

    private val listener: WebSocketEvents = WebSocketDefaultEvents()

    private var connectionAttempts = 0

     fun setConnectionStatusReceiver(receiver: MutableLiveData<ConnectionStatus>) {
        socketStatus = receiver
    }

    suspend fun connect() {
        try {
            Log.d(WEBSOCKET_TAG,"Connecting to websocket at $url...")

            session = client.webSocketSession(url)

            if(socketStatus != null)
                listener.onConnected(socketStatus!!)

            connectionAttempts = 0

            Log.d(WEBSOCKET_TAG,"Connected to websocket at $url")

        } catch (e: Exception) {
            // ... handle errors and reconnect
            if(socketStatus != null)
                listener.onDisconnected(socketStatus!!, e.message ?: "Unknown error")

            Log.d(WEBSOCKET_TAG,"Error: ${e.message}")

            connectionAttempts++

            reconnect()
        }
    }

    suspend fun reconnect() {
        job?.cancel()

        Log.d(WEBSOCKET_TAG,"Reconnecting to websocket in ${RECONNECT_DELAY}ms...")

        if(connectionAttempts <= RECONNECTION_ATTEMPTS) {
            job = scope.launch {
                disconnect()
                delay(RECONNECT_DELAY)
                connect()
            }
        }
        else
            connectionAttempts = 0
    }

    suspend fun disconnect() {
        Log.d(WEBSOCKET_TAG,"Closing websocket session...")

        session?.close()
        session = null

        if(socketStatus != null)
            listener.onDisconnected(socketStatus!!, "Disconnected successfully")
    }

     suspend fun <T> setDataReceiver(dataReceiver: MutableLiveData<T>) {

         var onReceive: ((String, MutableLiveData<T>) -> Unit)? = null

         when(dataReceiver.value) {
             is MutableList<*> -> onReceive = listener::onReceiveDetectedObject
             is Bitmap -> onReceive = listener::onReceiveCameraImage
         }

         if(session != null && onReceive != null) {
             session!!.incoming
                 .receiveAsFlow()
                 .filterIsInstance<Frame.Text>()
                 .filterNotNull()
                 .collect { data ->
                     val message = data.readText()

                     onReceive(message, dataReceiver)

                     Log.d(WEBSOCKET_TAG,"Received message: $message")
                 }
         }
     }

    suspend fun send(message: String) {
        Log.d(WEBSOCKET_TAG,"Sending message: $message")

        session?.send(Frame.Text(message))
    }
}