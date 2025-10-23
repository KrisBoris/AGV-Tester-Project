package com.example.agvtesterapp.ui

import android.os.Bundle
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.example.agvtesterapp.R
import com.example.agvtesterapp.database.ResultsDatabase
import com.example.agvtesterapp.databinding.ActivityMainBinding
import com.example.agvtesterapp.repository.Repository
import com.example.agvtesterapp.util.SocketType
import com.example.agvtesterapp.websocket.WebSocketClient

class MainActivity : AppCompatActivity() {

    /*
    Things to add:
    - prevent steering panel to go into sleep mode
    - add moving data from viewModel LiveData to the database after returning from the view
    - add connect all WebSockets button to the StartDriveFragment
    - Add clear database button to DriveResultsFragment
    - Add setting sockets ports in SettingsFragment + add method call setSocketIpAddress after changing it
        Disconnect all sockets and save new IP address in setIpAddress method
        Disconnect selected socket and save new IP address in setSocketPort method
    - Clear viewModel LiveData with detected objects after finished drive
     */

    private lateinit var binding: ActivityMainBinding
    lateinit var viewModel: ViewModel

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

//        val repository = Repository(applicationContext,
//            ResultsDatabase(this),
//            mapOf(
//                SocketType.CAMERA_IMAGE to WebSocketClient("${viewModel.getIpAddress()}:7891"),
//                SocketType.DETECTED_OBJECTS to WebSocketClient("${viewModel.getIpAddress()}:7892"),
//                SocketType.STEERING to WebSocketClient("${viewModel.getIpAddress()}:7893")
//            )
//        )

        val repository = Repository(applicationContext,
            ResultsDatabase(this),
            mapOf(
                SocketType.CAMERA_IMAGE to WebSocketClient(),
                SocketType.DETECTED_OBJECTS to WebSocketClient(),
                SocketType.STEERING to WebSocketClient()
            )
        )

        Log.i("DEBUG_TAG", "After repository creation")

        val viewModelProviderFactory = ViewModelProviderFactory(application, repository)
        viewModel = ViewModelProvider(this, viewModelProviderFactory).get(ViewModel::class.java)

        val navHostFragment = supportFragmentManager.findFragmentById(R.id.navHostFragment) as NavHostFragment
        val navController = navHostFragment.navController
        binding.bottomNavigationView.setupWithNavController(navController)

//        connectAllSockets()

        Log.i("DEBUG_TAG", "Main activity finished")
    }

    private fun connectAllSockets() {
        viewModel.connectSocket(SocketType.CAMERA_IMAGE, viewModel.cameraImage)
        viewModel.connectSocket(SocketType.DETECTED_OBJECTS, viewModel.detectedObjects)
        viewModel.connectSocket(SocketType.STEERING, MutableLiveData(Int))  // Pass something random to make him shut up (deleted this comment later)
    }

    override fun onDestroy() {
        super.onDestroy()

        viewModel.disconnectAllSockets()
    }
}