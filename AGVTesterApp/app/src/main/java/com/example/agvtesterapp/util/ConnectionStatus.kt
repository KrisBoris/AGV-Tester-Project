package com.example.agvtesterapp.util

/**
 * Sealed class containing all possible connections statuses
 * @param message message passed while setting the connection status
 */
sealed class ConnectionStatus(val message: String? = null) {

    /**
     * Status representing client's successful connection to the server
     */
    class Connected: ConnectionStatus()

    /**
     * Status representing that client isn't connected to the server
     * @param message optional explanation why disconnection occurred
     */
    class Disconnected(message: String? = null): ConnectionStatus(message)
}