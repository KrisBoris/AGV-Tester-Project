package com.example.agvtesterapp.websocket

import android.util.Log
import androidx.lifecycle.MutableLiveData
import com.example.agvtesterapp.models.Twist
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import com.google.gson.Gson
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
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.plus
import java.util.concurrent.TimeUnit

/**
 * Class containing WebSocket client's data and methods necessary for managing
 * client's communication with server (e.g. establishing connection with server,
 * receiving data from server, sending messages to server)
 * @param url server IP address (IPv4 protocol)
 * @param socketStatus reference to container holding connection status with server
 */
 class WebSocketClient(
     private var url: String? = null,
     private var socketStatus: MutableLiveData<ConnectionStatus>? = null
) {
    companion object {
        /**
         * Default IP address
         */
        const val IP_ADDRESS = "192.168.45.28"

        /**
         * Default camera image socket port
         */
        const val CAMERA_SOCKET_PORT = "7891"

        /**
         * Default detected objects socket port
         */
        const val OBJECTS_SOCKET_PORT = "7892"

        /**
         * Default steering socket port
         */
        const val STEERING_SOCKET_PORT = "7893"

        /**
         * Tag for WebSocket related logs
         */
        const val WEBSOCKET_TAG = "WEBSOCKET_TAG"

        /**
         * Default WebSocket ping interval
         */
        const val PING_INTERVAL = 10_000L   // 10s

        /**
         * Default delay period between reconnection attempts
         */
        const val RECONNECT_DELAY = 5_000L

        /**
         * Default number of reconnection attempts
         */
        const val RECONNECTION_ATTEMPTS = 3
    }

    /**
     * WebSocket HTTP client object that maintains a heartbeat connection with server
     * It uses Ktor [HttpClient] based on the [OkHttp] engine (the network layer)
     */
    private val client = HttpClient(OkHttp) {
        engine {
            config {
                // Configures the WebSocket ping interval
                // Client automatically sends a “ping” frame to the server every
                // PING_INTERVAL milliseconds to keep the connection alive and detect timeouts
                pingInterval(PING_INTERVAL, TimeUnit.MILLISECONDS)
            }
        }
        // Enables logging for all HTTP/WebSocket activity
        install(Logging)
        // Installs the WebSocket feature so this client can open and manage WebSocket sessions
        install(WebSockets)
    }

    /**
     * Scope in which coroutine responsible for running
     * client's communication with server is launched
     */
    private val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() +
            CoroutineExceptionHandler { _, throwable ->
                Log.e(WEBSOCKET_TAG, "Error: ${throwable.message}")
            }

    /**
     * Coroutine object in which client's communication with server is managed
     */
    private var job: Job? = null

    /**
     * WebSocket session object
     * (holds the active WebSocket connection once it’s established)
     */
    private var session: WebSocketSession? = null

    /**
     * Class containing methods called upon designated WebSocket client's events
     */
    private val listener: WebSocketEvents = WebSocketDefaultEvents()

    /**
     * Counter of client's connection attempts to server
     */
    private var connectionAttempts = 0


    /**
     * Listens for incoming messages from the server and passes them to the specified container
     * @param socketType socket type
     * @param dataReceiver reference to the container for data received from server
     */
    private suspend fun <T> listenForMessages(socketType: SocketType, dataReceiver: MutableLiveData<T>? = null) {

        if (dataReceiver != null) {
            // Based on socket type, a different listener method for managing incoming data is chosen
            val onReceive: ((String, MutableLiveData<T>) -> Unit)? = when (socketType) {
                SocketType.CAMERA_IMAGE -> listener::onReceiveCameraImage
                SocketType.DETECTED_OBJECTS -> listener::onReceiveDetectedObject
                else -> null
            }

            try {
                session?.incoming   // Channel receiving all incoming messages from the server
                    ?.receiveAsFlow()   // Converts the incoming stream into a Flow, enabling coroutine-based collection
                    ?.filterIsInstance<Frame.Text>()    // Ignores binary/ping/pong frames and processes only text frames
                    ?.collect { data ->     // Runs continuously until the flow ends (i.e., server closes connection)
                        val message = data.readText()
                        onReceive?.invoke(message, dataReceiver)
                    }

                // Flow finished normally (server closed connection)
                socketStatus?.let { listener.onDisconnected(it, "Server closed connection") }
            }
            catch (e: Exception) {
                // Server disconnected unexpectedly
                Log.e(WEBSOCKET_TAG, "WebSocket error: ${e.message}")
                socketStatus?.let { listener.onDisconnected(it, "Server disconnected: ${e.message}") }
            }
            finally {
                Log.i(WEBSOCKET_TAG, "WebSocket listener stopped")
            }
        }
    }

    /**
     * Sets IP address of the server to which client is connection to
     * @param address IP address (IPv4 protocol)
     */
    fun setIpAddress(address: String) {
        url = address
    }

    /**
     * Assigns reference to container holding connection status with server
     * @param receiver reference to container holding connection status with server
     */
    fun setConnectionStatusReceiver(receiver: MutableLiveData<ConnectionStatus>) {
        socketStatus = receiver
    }

    /**
     * Connects WebSocket client to the WebSocket server specified by client's IP address
     * @param socketType socket type
     * @param dataReceiver reference to the container for data received from server
     */
    suspend fun <T> connect(socketType: SocketType, dataReceiver: MutableLiveData<T>? = null) {
        try {
            if (url != null) {
                session = client.webSocketSession(url!!)

                if(socketStatus != null)
                    listener.onConnected(socketStatus!!)

                connectionAttempts = 0
                Log.i(WEBSOCKET_TAG,"Connected to WebSocket server at $url")

                if (dataReceiver != null) {
                    // Launches coroutine and listens for messages continuously
                    scope.launch {
                        listenForMessages(socketType, dataReceiver)
                    }
                }
            }


        } catch (e: Exception) {
            // Handle errors and reconnect
            if(socketStatus != null)
                listener.onDisconnected(socketStatus!!, e.message ?: "Unknown error")

            Log.e(WEBSOCKET_TAG,"Error: ${e.message}")
            connectionAttempts++
            reconnect(socketType, dataReceiver)
        }
    }

    /**
     * Reconnects WebSocket client to the WebSocket server specified by client's IP address
     * @param socketType socket type
     * @param dataReceiver reference to the container for data received from server
     */
    suspend fun <T> reconnect(socketType: SocketType, dataReceiver: MutableLiveData<T>? = null) {
        // Cancel the coroutine in which client connection is running
        job?.cancel()

        if(connectionAttempts <= RECONNECTION_ATTEMPTS) {
            // Launch new coroutine in which client communication with server will be managed
            job = scope.launch {
                disconnect()
                delay(RECONNECT_DELAY)
                connect(socketType, dataReceiver)
            }
        }
        else
            connectionAttempts = 0
    }

    /**
     * Disconnects WebSocket client from WebSocket server
     */
    suspend fun disconnect() {
        session?.close()
        session = null
        Log.i(WEBSOCKET_TAG,"WebSocket session closed")

        if(socketStatus != null)
            listener.onDisconnected(socketStatus!!, "Disconnected successfully")
    }

    /**
     * Sends steering command to the WebSocket server
     * @param cmd steering command corresponding to ROS geometry_msgs/Twist message type
     */
    suspend fun send(cmd: Twist) {
        try {
            // Parse Twist message to JSON format
            val jsonString = Gson().toJson(cmd)

            // Send JSON message to WebSocket server
            session?.send(Frame.Text(jsonString))

        } catch (e: Exception) {
            Log.e(WEBSOCKET_TAG, "Error while sending steering command: ${e.message}")
        }
    }
}