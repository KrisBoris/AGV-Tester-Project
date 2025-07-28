package com.example.agvtesterapp.util

sealed class ConnectionStatus(val message: String? = null) {
    class Connected: ConnectionStatus()
    class Disconnected(message: String? = null): ConnectionStatus(message)
}