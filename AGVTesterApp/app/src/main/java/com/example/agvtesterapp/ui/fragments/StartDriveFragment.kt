package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.view.View
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.lifecycle.MutableLiveData
import androidx.navigation.fragment.findNavController
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentStartDriveBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.example.agvtesterapp.util.ConnectionStatus
import com.example.agvtesterapp.util.SocketType

/**
 * Start drive fragment's class - used to manage fragments UI functions like
 * displaying connection status, connecting, reconnecting and disconnecting
 * WebSocket clients and navigating to the steering panel
 */
class StartDriveFragment : Fragment(R.layout.fragment_start_drive) {

    /**
     * View Binding object - allows to safely access view
     */
    private lateinit var binding: FragmentStartDriveBinding

    /**
     * Reference to [ViewModel] instance - allows for sharing data and methods between activities and fragments
     */
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentStartDriveBinding.bind(view)

        // Gets reference to the MainActivity's ViewModel instance
        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            setupConnectionStatusObserver(SocketType.CAMERA_IMAGE, tvCameraConnectionStatus)
            setupConnectionStatusObserver(SocketType.DETECTED_OBJECTS, tvDataConnectionStatus)
            setupConnectionStatusObserver(SocketType.STEERING, tvSteeringConnectionStatus)

            btnRefreshDataConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.DETECTED_OBJECTS, viewModel.detectedObjects)
            }
            btnRefreshCameraConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.CAMERA_IMAGE, viewModel.cameraImage)
            }
            btnRefreshSteeringConnection.setOnClickListener {
                viewModel.reconnectSocket(SocketType.STEERING, MutableLiveData(Int))    // Steering socket doesn't receive any data
            }

            btnConnectAllSockets.setOnClickListener {
                connectAllSockets()
            }
            btnDisconnectAllSockets.setOnClickListener {
                viewModel.disconnectAllSockets()
                // Clears data received while sockets were connected
                viewModel.clearWebsocketDataContainers()
            }
            btnStartDrive.setOnClickListener {
                connectAllSockets()
                // Navigate to the steering panel
                findNavController().navigate(R.id.action_startDriveFragment_to_steeringPanelFragment)
            }
        }
    }

    /**
     * Attaches given [TextView] to designated socket status.
     * Whenever socket status changes, the appropriate message is displayed
     * @param socket socket type
     * @param textView [TextView] object where connection status will be displayed
     */
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

    /**
     * Connects all WebSocket clients to respective WebSocket servers
     */
    private fun connectAllSockets() {
        viewModel.disconnectAllSockets()
        viewModel.connectSocket(SocketType.CAMERA_IMAGE, viewModel.cameraImage)
        viewModel.connectSocket(SocketType.DETECTED_OBJECTS, viewModel.detectedObjects)
        viewModel.connectSocket(SocketType.STEERING, MutableLiveData(Int))  // Steering socket doesn't receive any data
    }
}