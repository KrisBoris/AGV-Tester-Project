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
import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import kotlinx.coroutines.plus

/**
 * Steering panel fragment's class - used to manage fragments UI functions like
 * displaying camera image, sending steering commands based on which steering button was pressed
 * and moving all detected objects to the database after leaving the fragment
 */
class SteeringPanelFragment : Fragment(R.layout.fragment_steering_panel) {

    /**
     * View Binding object - allows to safely access view
     */
    private lateinit var binding: FragmentSteeringPanelBinding

    /**
     * Reference to [ViewModel] instance - allows for sharing data and methods between activities and fragments
     */
    private lateinit var viewModel: ViewModel

    /**
     * [Twist] command used to set AGV's velocity while driving forward
     */
    private val forwardVelocity = Twist(Vector3(0.7, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    /**
     * [Twist] command used to set AGV's velocity while driving backward
     */
    private val backwardVelocity = Twist(Vector3(-0.7, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    /**
     * [Twist] command used to set AGV's velocity while turning left
     */
    private val leftTurnVelocity = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 1.0))
    /**
     * [Twist] command used to set AGV's velocity while turning right
     */
    private val rightTurnVelocity = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, -1.0))

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSteeringPanelBinding.bind(view)

        // Gets reference to the MainActivity's ViewModel instance
        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            // Coroutine scope used for IO operations
            val scope = CoroutineScope(Dispatchers.IO) + SupervisorJob() + CoroutineExceptionHandler {_, throwable ->
                Log.e(ViewModel.APP_TAG, "Error: ${throwable.message}")}

            // Whenever new image is received from WebSocket server and saved to the container
            // is will be displayed in the ImageView object
            viewModel.cameraImage.observe(viewLifecycleOwner) { image ->
                agvImageView.setImageBitmap(image)
            }

            setButtonOnTouchListener(btnForward, scope, forwardVelocity)
            setButtonOnTouchListener(btnBackward, scope, backwardVelocity)
            setButtonOnTouchListener(btnLeft, scope, leftTurnVelocity)
            setButtonOnTouchListener(btnRight, scope, rightTurnVelocity)

            btnFinish.setOnClickListener {
                cleanupAndLeave()
            }
        }

        // Called when the back button is pressed
        requireActivity().onBackPressedDispatcher.addCallback(viewLifecycleOwner) {
            cleanupAndLeave()
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

    /**
     * Sets button on touch listener. Specifies what action needs to be performed after
     * the button is pressed and after it is released
     * @param button button to set (XML object)
     * @param scope coroutine scope in which IO operations related to buttons are performed
     * @param cmd steering command corresponding to ROS geometry_msgs/Twist message type
     */
    @SuppressLint("ClickableViewAccessibility")
    private fun setButtonOnTouchListener(button: Button, scope: CoroutineScope, cmd: Twist) {
        button.setOnTouchListener { view, event ->
            when (event?.action) {

                // Action performed after button was pressed
                MotionEvent.ACTION_DOWN -> {
                    scope.launch {
                        while (button.isPressed) {
                            // Sends the steering command every 20ms
                            // It is imperative for ROS to receive the command frequently,
                            // otherwise it will stop the vehicle
                            viewModel.sendSteeringCommand(cmd)
                            kotlinx.coroutines.delay(20)
                        }
                    }
                }

                // Action performed after button was released
                MotionEvent.ACTION_UP -> {
                    scope.launch {
                        // Stops the vehicle immediately after releasing the button
                        viewModel.sendSteeringCommand(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
                    }
                }
            }

            view?.onTouchEvent(event) ?: true
        }
    }

    /**
     * Inserts all objects detected during drive into database
     * and clears viewModel's list of detected objects afterwards
     */
    private fun moveObjectsToDatabase() {
        // Move detected objects from viewModel MutableList to the database
        if (viewModel.detectedObjects.value != null) {
            for (detectedObject in viewModel.detectedObjects.value!!)
                viewModel.addDetectedObject(detectedObject)

            // Clear all detected objects from viewModel MutableList
            viewModel.detectedObjects.value?.clear()
        }
    }

    /**
     * Performs operations required before leaving the view:
     * disconnects all sockets, saves all detected objects in database,
     * clears WebSocket data containers and returns to the previous view
     */
    private fun cleanupAndLeave() {
        // Disconnects all sockets to no longer receive data from them
        viewModel.disconnectAllSockets()
        // Saves all objects detected during drive to the database
        moveObjectsToDatabase()
        viewModel.clearWebsocketDataContainers()
        findNavController().popBackStack()
    }
}