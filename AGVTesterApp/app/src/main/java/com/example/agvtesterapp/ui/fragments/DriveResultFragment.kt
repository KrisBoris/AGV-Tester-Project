package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.view.View
import androidx.fragment.app.Fragment
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.agvtesterapp.R
import com.example.agvtesterapp.adapters.DriveResultsAdapter
import com.example.agvtesterapp.databinding.FragmentDriveResultBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel

/**
 * Drive results fragment class - used to manage fragments UI functions like
 * Displaying all detected objects from the database and clearing database
 */
class DriveResultFragment : Fragment(R.layout.fragment_drive_result) {

    /**
     * View Binding object - allows to safely access view
     */
    private lateinit var binding: FragmentDriveResultBinding

    /**
     * Reference to [ViewModel] instance - allows for sharing data and methods between activities and fragments
     */
    private lateinit var viewModel: ViewModel

    /**
     * RecyclerView adapter
     */
    private lateinit var driveResultsAdapter: DriveResultsAdapter

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentDriveResultBinding.bind(view)

        // Gets reference to the MainActivity's ViewModel instance
        viewModel =(activity as MainActivity).viewModel
        setupRecyclerView()

        // Whenever the content of the database is changed,
        // the new list of objects is passed to the adapter
        viewModel.getResults().observe(viewLifecycleOwner) { detectedObjects ->
            driveResultsAdapter.addDetectedObjects(detectedObjects)
        }

        binding.apply {
            btnClearDriveResults.setOnClickListener {
                viewModel.clearResults()
            }
        }
    }

    /**
     * Sets up RecyclerView's adapter and layout
     */
    private fun setupRecyclerView() {
        driveResultsAdapter = DriveResultsAdapter()
        binding.rvDriveResult.apply {
            adapter = driveResultsAdapter
            layoutManager = LinearLayoutManager(activity)
        }
    }
}