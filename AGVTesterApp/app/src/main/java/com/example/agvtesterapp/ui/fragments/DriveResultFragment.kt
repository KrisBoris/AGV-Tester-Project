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

class DriveResultFragment : Fragment(R.layout.fragment_drive_result) {

    private lateinit var binding: FragmentDriveResultBinding
    private lateinit var viewModel: ViewModel
    private lateinit var adapter: DriveResultsAdapter
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentDriveResultBinding.bind(view)

        viewModel =(activity as MainActivity).viewModel
        setupRecyclerView()

        viewModel.getResults().observe(viewLifecycleOwner) { detectedObjects ->
            adapter.addDetectedObjects(detectedObjects)
        }
    }

    private fun setupRecyclerView() {
        adapter = DriveResultsAdapter()
        binding.rvDriveResult.apply {
            adapter = adapter
            layoutManager = LinearLayoutManager(activity)
        }
    }
}