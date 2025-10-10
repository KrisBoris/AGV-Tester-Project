package com.example.agvtesterapp.ui.fragments

import android.os.Bundle
import android.view.View
import androidx.fragment.app.Fragment
import com.example.agvtesterapp.R
import com.example.agvtesterapp.databinding.FragmentSettingsBinding
import com.example.agvtesterapp.ui.MainActivity
import com.example.agvtesterapp.ui.ViewModel
import com.google.android.material.snackbar.Snackbar

class SettingsFragment : Fragment(R.layout.fragment_settings) {

    private lateinit var binding: FragmentSettingsBinding
    private lateinit var viewModel: ViewModel

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding = FragmentSettingsBinding.bind(view)

        viewModel = (activity as MainActivity).viewModel

        binding.apply {

            btnSaveIpAddress.setOnClickListener {
                if (viewModel.setIpAddress(etSetIpAddress.text.toString())) {
                    Snackbar.make(view, "IP address set to ${etSetIpAddress.text}", Snackbar.LENGTH_LONG).show()
                    etSetIpAddress.text.clear()

                    viewModel.disconnectAllSockets()
                }
                else
                    Snackbar.make(view, "Invalid IP address", Snackbar.LENGTH_LONG).show()
            }
        }
    }
}