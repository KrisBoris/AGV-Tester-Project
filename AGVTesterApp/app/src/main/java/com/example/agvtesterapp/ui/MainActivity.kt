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

/**
 * Class that contains application's MainActivity - used to set up
 * the most important components of the application ([ViewModel], [Repository] etc.)
 */
class MainActivity : AppCompatActivity() {

    /**
     * View Binding object - allows to safely access view
     */
    private lateinit var binding: ActivityMainBinding

    /**
     * [ViewModel] instance - allows for sharing data and methods between activities and fragments
     */
    lateinit var viewModel: ViewModel

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Repository setup
        val repository = Repository(applicationContext,
            ResultsDatabase(this),
            mapOf(
                // Creates separate WebSocketClient instances - one for each socket type
                SocketType.CAMERA_IMAGE to WebSocketClient(),
                SocketType.DETECTED_OBJECTS to WebSocketClient(),
                SocketType.STEERING to WebSocketClient()
            )
        )

        // ViewModel setup
        val viewModelProviderFactory = ViewModelProviderFactory(application, repository)
        viewModel = ViewModelProvider(this, viewModelProviderFactory).get(ViewModel::class.java)

        // Navigation controller setup
        val navHostFragment = supportFragmentManager.findFragmentById(R.id.navHostFragment) as NavHostFragment
        val navController = navHostFragment.navController
        binding.bottomNavigationView.setupWithNavController(navController)
    }

    override fun onDestroy() {
        super.onDestroy()

        // Disconnects all WebSocket clients once the MainActivity is destroyed
        viewModel.disconnectAllSockets()
    }
}