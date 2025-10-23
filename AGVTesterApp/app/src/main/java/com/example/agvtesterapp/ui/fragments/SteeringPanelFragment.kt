package com.example.agvtesterapp.ui.fragments

import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.widget.Button
import androidx.activity.addCallback
import androidx.fragment.app.Fragment
import androidx.navigation.fragment.findNavController
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentSteeringPanelBinding
import com.example.agvtesterapp.models.Twist
import com.example.agvtesterapp.models.Vector3
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

    private val forwardVelocity = Twist(Vector3(0.7, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    private val backwardVelocity = Twist(Vector3(-0.7, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    private val leftTurnVelocity = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 1.0))
    private val rightTurnVelocity = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, -1.0))

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSteeringPanelBinding.bind(view)

        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() + CoroutineExceptionHandler {_, throwable ->
                Log.d(WebSocketClient.WEBSOCKET_TAG, "Error: ${throwable.message}")}

            viewModel.cameraImage.observe(viewLifecycleOwner) { image ->
                agvImageView.setImageBitmap(image)
            }

            setButtonOnTouchListener(btnForward, scope, forwardVelocity)
            setButtonOnTouchListener(btnBackward, scope, backwardVelocity)
            setButtonOnTouchListener(btnLeft, scope, leftTurnVelocity)
            setButtonOnTouchListener(btnRight, scope, rightTurnVelocity)

            btnFinish.setOnClickListener {
                moveObjectsToDatabase()
                findNavController().popBackStack()
            }
        }

        requireActivity().onBackPressedDispatcher.addCallback(viewLifecycleOwner) {
            // Called when the back button is pressed
            moveObjectsToDatabase()
            findNavController().popBackStack()
        }
    }

    override fun onResume() {
        super.onResume()
        // Keeps the screen constantly on while in the steering panel
        requireActivity().window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onPause() {
        super.onPause()
        // Clears the flag to no longer keep the screen constantly on
        requireActivity().window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setButtonOnTouchListener(button: Button, scope: CoroutineScope, cmd: Twist) {
        button.setOnTouchListener { view, event ->
            when (event?.action) {
                MotionEvent.ACTION_DOWN -> {
                    scope.launch {
                        while (button.isPressed) {
                            viewModel.sendSteeringCommand(SocketType.STEERING, cmd)
                            // Sends the steering command every 50ms
                            // It is imperative for ROS to receive the command frequently, otherwise it will stop the vehicle
                            kotlinx.coroutines.delay(50)
                        }
                    }
                }

                MotionEvent.ACTION_UP -> {
                    scope.launch {
                        // Stops the vehicle immediately after releasing the button
                        viewModel.sendSteeringCommand(SocketType.STEERING, Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
                    }
                }
            }

            view?.onTouchEvent(event) ?: true
        }
    }

    private fun moveObjectsToDatabase() {
        // Move detected objects from viewModel MutableList to the database
        if (viewModel.detectedObjects.value != null) {
            for (detectedObject in viewModel.detectedObjects.value!!) {
                Log.d(WebSocketClient.WEBSOCKET_TAG, "Adding object to the list: ${detectedObject.name}, count: ${detectedObject.count}")
                viewModel.addDetectedObject(detectedObject)
            }
            viewModel.detectedObjects.value?.clear()
        }
    }
}