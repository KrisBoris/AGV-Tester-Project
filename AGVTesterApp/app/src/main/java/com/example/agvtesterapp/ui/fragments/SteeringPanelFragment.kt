package com.example.agvtesterapp.ui.fragments

import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import androidx.fragment.app.Fragment
import androidx.lifecycle.Observer
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentSteeringPanelBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient
import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import kotlinx.coroutines.plus

class SteeringPanelFragment : Fragment(R.layout.fragment_steering_panel) {

    private lateinit var binding: FragmentSteeringPanelBinding
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSteeringPanelBinding.bind(view)

        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() + CoroutineExceptionHandler {_, throwable ->
                Log.d(WebSocketClient.WEBSOCKET_TAG, "Error: ${throwable.message}")}

            scope.launch {
                viewModel.cameraImage.observe(viewLifecycleOwner, Observer { image ->
                    agvImageView.setImageBitmap(image)
                })
            }

            setButtonOnTouchListener(btnForward, scope, "fwd")
            setButtonOnTouchListener(btnLeft, scope, "lft")
            setButtonOnTouchListener(btnBackward, scope, "rgt")
            setButtonOnTouchListener(btnRight, scope, "bwd")

            btnFinish.setOnClickListener {

                viewModel.reconnectSocket(SocketType.DETECTED_OBJECTS)
                viewModel.reconnectSocket(SocketType.CAMERA_IMAGE)

                if(viewModel.detectedObjects.value != null) {
                    for(detectedObject in viewModel.detectedObjects.value!!)
                        viewModel.addDetectedObject(detectedObject)
                }

                findNavController().popBackStack()
            }
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setButtonOnTouchListener(button: Button, scope: CoroutineScope, message: String) {
        button.setOnTouchListener { view, event ->
            when (event?.action) {
                MotionEvent.ACTION_DOWN -> {
                    scope.launch {
                        viewModel.sendSteeringCommand(SocketType.STEERING, "/$message")
                    }
                }

                MotionEvent.ACTION_UP -> {
                    scope.launch {
                        viewModel.sendSteeringCommand(SocketType.STEERING, "/stp")
                    }
                }
            }

            view?.onTouchEvent(event) ?: true
        }
    }
}