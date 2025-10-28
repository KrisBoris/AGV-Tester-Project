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

class SettingsFragment : Fragment(R.layout.fragment_settings) {

    private lateinit var binding: FragmentSettingsBinding
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSettingsBinding.bind(view)

        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            btnSaveChanges.setOnClickListener {
                if (etSetIpAddress.text.toString().isNotEmpty()) {
                    if (viewModel.setIpAddress(etSetIpAddress.text.toString())) {

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
                if (etSetCameraPort.text.toString().isNotEmpty()) {
                    if (viewModel.setSocketPort(SocketType.CAMERA_IMAGE, etSetCameraPort.text.toString())) {

                        setSocketsIpAddress(SocketType.CAMERA_IMAGE)

                        Snackbar.make(view, "Camera socket port set to ${etSetCameraPort.text}", Snackbar.LENGTH_LONG).show()
                        etSetCameraPort.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid port number", Snackbar.LENGTH_LONG).show()
                }
                if (etSetDetectedObjectsPort.text.toString().isNotEmpty()) {
                    if (viewModel.setSocketPort(SocketType.DETECTED_OBJECTS, etSetDetectedObjectsPort.text.toString())) {

                        setSocketsIpAddress(SocketType.DETECTED_OBJECTS)

                        Snackbar.make(view, "Detected objects socket port set to ${etSetDetectedObjectsPort.text}", Snackbar.LENGTH_LONG).show()
                        etSetDetectedObjectsPort.text.clear()
                    }
                    else
                        Snackbar.make(view, "Invalid port number", Snackbar.LENGTH_LONG).show()
                }
                if (etSetSteeringPort.text.toString().isNotEmpty()) {
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

    private fun setSocketsIpAddress(socket: SocketType) {
        viewModel.disconnectSocket(socket)
        val url = "${viewModel.getIpAddress()}:${viewModel.getSocketPort(socket)}"
        viewModel.setSocketIpAddress(socket, url)
    }
}