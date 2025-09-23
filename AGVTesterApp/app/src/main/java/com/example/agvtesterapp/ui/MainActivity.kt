package com.example.agvtesterapp.ui

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
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
    - var for drive status (is drive started) + not allowing to change IP during drive +
        after IP change connect to new IP
    - add default image in case of losing camera connection
    - add serialization
     */

    private lateinit var binding: ActivityMainBinding
    lateinit var viewModel: ViewModel

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val repository = Repository(applicationContext,
            ResultsDatabase(this),
            mapOf(
                SocketType.CAMERA_IMAGE to WebSocketClient("${WebSocketClient.IP_ADDRESS}:7891"),
                SocketType.DETECTED_OBJECTS to WebSocketClient("${WebSocketClient.IP_ADDRESS}:7892"),
                SocketType.STEERING to WebSocketClient("${WebSocketClient.IP_ADDRESS}:7893")
            )
        )

        val viewModelProviderFactory = ViewModelProviderFactory(application, repository)
        viewModel = ViewModelProvider(this, viewModelProviderFactory).get(ViewModel::class.java)

        val navHostFragment = supportFragmentManager.findFragmentById(R.id.navHostFragment) as NavHostFragment
        val navController = navHostFragment.navController
        binding.bottomNavigationView.setupWithNavController(navController)

        connectAllSockets()
    }

    private fun connectAllSockets() {
        for(socket in SocketType.entries)
            viewModel.connectSocket(socket)
    }

    override fun onDestroy() {
        super.onDestroy()

        disconnectAllSockets()
    }

    private fun disconnectAllSockets() {
        for(socket in SocketType.entries)
            viewModel.disconnectSocket(socket)
    }
}