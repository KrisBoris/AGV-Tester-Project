package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.view.View
import androidx.fragment.app.Fragment
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentSettingsBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.example.agvtesterapp.util.SocketType
import com.google.android.material.snackbar.Snackbar

/**
 * Settings fragment class - used to manage fragments UI functions like
 * setting parameters like WebSocket server IP address and ports' numbers
 * for specific sockets
 */
class SettingsFragment : Fragment(R.layout.fragment_settings) {

    /**
     * View Binding object - allows to safely access view
     */
    private lateinit var binding: FragmentSettingsBinding

    /**
     * Reference to [ViewModel] instance - allows for sharing data and methods between activities and fragments
     */
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSettingsBinding.bind(view)

        // Gets reference to the MainActivity's ViewModel instance
        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            btnSaveChanges.setOnClickListener {

                // If new IP address was entered
                if (etSetIpAddress.text.toString().isNotEmpty()) {
                    // If new IP address is valid (for IPv4 protocol)
                    if (viewModel.setIpAddress(etSetIpAddress.text.toString())) {
                        // Disconnect all sockets and set new IP address for each one of them
                        viewModel.disconnectAllSockets()
                        for (socket in SocketType.entries) {
                            val url = "${viewModel.getIpAddress()}:${viewModel.getSocketPort(socket)}"
                            viewModel.setSocketIpAddress(socket, url)
                        }
                        Snackbar.make(view, "IP address set to ${etSetIpAddress.text}", Snackbar.LENGTH_LONG).show()
                        etSetIpAddress.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid IP address", Snackbar.LENGTH_LONG).show()
                }

                // If new camera image port number was entered
                if (etSetCameraPort.text.toString().isNotEmpty()) {
                    // If port number is valid
                    if (viewModel.setSocketPort(SocketType.CAMERA_IMAGE, etSetCameraPort.text.toString())) {
                        setSocketsIpAddress(SocketType.CAMERA_IMAGE)

                        Snackbar.make(view, "Camera socket port set to ${etSetCameraPort.text}", Snackbar.LENGTH_LONG).show()
                        etSetCameraPort.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid port number", Snackbar.LENGTH_LONG).show()
                }

                // If new detected objects port was entered
                if (etSetDetectedObjectsPort.text.toString().isNotEmpty()) {
                    // If port number is valid
                    if (viewModel.setSocketPort(SocketType.DETECTED_OBJECTS, etSetDetectedObjectsPort.text.toString())) {
                        setSocketsIpAddress(SocketType.DETECTED_OBJECTS)

                        Snackbar.make(view, "Detected objects socket port set to ${etSetDetectedObjectsPort.text}", Snackbar.LENGTH_LONG).show()
                        etSetDetectedObjectsPort.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid port number", Snackbar.LENGTH_LONG).show()
                }

                // If new steering port was entered
                if (etSetSteeringPort.text.toString().isNotEmpty()) {
                    // If port number is valid
                    if (viewModel.setSocketPort(SocketType.STEERING, etSetSteeringPort.text.toString())) {
                        setSocketsIpAddress(SocketType.STEERING)

                        Snackbar.make(view, "Steering socket port set to ${etSetSteeringPort.text}", Snackbar.LENGTH_LONG).show()
                        etSetSteeringPort.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid port number", Snackbar.LENGTH_LONG).show()
                }
            }
        }
    }

    /**
     * Disconnects WebSocket client and sets up new IP address for given socket type
     * @param socket socket type
     */
    private fun setSocketsIpAddress(socket: SocketType) {
        viewModel.disconnectSocket(socket)
        val url = "${viewModel.getIpAddress()}:${viewModel.getSocketPort(socket)}"
        viewModel.setSocketIpAddress(socket, url)
    }
}