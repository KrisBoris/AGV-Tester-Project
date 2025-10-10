package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.Observer
import androidx.navigation.fragment.findNavController
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentStartDriveBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient
import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.plus

class StartDriveFragment : Fragment(R.layout.fragment_start_drive) {

    private lateinit var binding: FragmentStartDriveBinding
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentStartDriveBinding.bind(view)

        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            setupConnectionStatusObserver(SocketType.DETECTED_OBJECTS, tvDataConnectionStatus)
            setupConnectionStatusObserver(SocketType.CAMERA_IMAGE, tvCameraConnectionStatus)
            setupConnectionStatusObserver(SocketType.STEERING, tvSteeringConnectionStatus)

            btnRefreshDataConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.DETECTED_OBJECTS, viewModel.detectedObjects)
            }
            btnRefreshCameraConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.CAMERA_IMAGE, viewModel.cameraImage)
            }
            btnRefreshSteeringConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.STEERING, MutableLiveData(Int))
            }

            btnStartDrive.setOnClickListener {
                findNavController().navigate(R.id.action_startDriveFragment_to_steeringPanelFragment)
            }
        }
    }

    private fun setupConnectionStatusObserver(socket: SocketType, textView: TextView) {
        viewModel.socketsStatus[socket]?.observe(viewLifecycleOwner) { status ->
            when (status) {
                is ConnectionStatus.Connected -> {
                    textView.setTextColor(ContextCompat.getColor(requireContext(), R.color.green))
                    textView.text = getString(R.string.agv_connection_status_connected)
                }

                is ConnectionStatus.Disconnected -> {
                    textView.setTextColor(ContextCompat.getColor(requireContext(), R.color.red))
                    textView.text = getString(R.string.agv_connection_status_disconnected)
                }

                else -> {
                    textView.setTextColor(ContextCompat.getColor(requireContext(), R.color.red))
                    textView.text = getString(R.string.agv_connection_status_error)
                }
            }
        }
    }
}