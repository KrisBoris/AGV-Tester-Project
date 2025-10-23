package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.util.Log
import android.view.View
import androidx.fragment.app.Fragment
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.agvtesterapp.R
import com.example.agvtesterapp.adapters.DriveResultsAdapter
import com.example.agvtesterapp.databinding.FragmentDriveResultBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.example.agvtesterapp.websocket.WebSocketClient

class DriveResultFragment : Fragment(R.layout.fragment_drive_result) {

    private lateinit var binding: FragmentDriveResultBinding
    private lateinit var viewModel: ViewModel
    private lateinit var driveResultsAdapter: DriveResultsAdapter
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentDriveResultBinding.bind(view)

        viewModel =(activity as MainActivity).viewModel
        setupRecyclerView()
        Log.d(WebSocketClient.WEBSOCKET_TAG, "Setup recycler view")

        viewModel.getResults().observe(viewLifecycleOwner) { detectedObjects ->
            detectedObjects.forEach { detObj ->
                Log.d(WebSocketClient.WEBSOCKET_TAG, "RecyclerView: ${detObj.name}, count: ${detObj.count}")
            }
            driveResultsAdapter.addDetectedObjects(detectedObjects)
        }

        binding.apply {
            btnClearDriveResults.setOnClickListener {
                viewModel.clearResults()
            }
        }
    }

    private fun setupRecyclerView() {
        driveResultsAdapter = DriveResultsAdapter()
        binding.rvDriveResult.apply {
            adapter = driveResultsAdapter
            layoutManager = LinearLayoutManager(activity)
        }
    }
}