package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
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
import kotlinx.coroutines.launch
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

            val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() + CoroutineExceptionHandler { _, throwable ->
                Log.d(WebSocketClient.WEBSOCKET_TAG, "Error: ${throwable.message}")}

//            btnRefreshDataConnection.setOnClickListener {
//                viewModel.reconnectSocket(SocketType.DETECTED_OBJECTS)
//            }
//            btnRefreshCameraConnection.setOnClickListener {
//                viewModel.reconnectSocket(SocketType.CAMERA_IMAGE)
//            }
//            btnRefreshSteeringConnection.setOnClickListener {
//                viewModel.reconnectSocket(SocketType.STEERING)
//            }

            btnStartDrive.setOnClickListener {

                findNavController().navigate(R.id.action_startDriveFragment_to_steeringPanelFragment)

//                // If steering socket is connected go to steering panel
//                if(viewModel.socketsStatus[SocketType.DETECTED_OBJECTS]?.value is ConnectionStatus.Connected) {
//
//                    // Start receiving data from server (detected objects and camera image)
////                    viewModel.setDataReceiver(SocketType.DETECTED_OBJECTS, viewModel.detectedObjects)
////                    viewModel.setDataReceiver(SocketType.CAMERA_IMAGE, viewModel.cameraImage)
//
//                    findNavController().navigate(R.id.action_startDriveFragment_to_steeringPanelFragment)
//                }
            }
        }
    }

    private fun setupConnectionStatusObserver(socket: SocketType, textView: TextView) {
        viewModel.socketsStatus[socket]?.observe(viewLifecycleOwner, Observer { status ->
            when(status) {
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
        })
    }
}