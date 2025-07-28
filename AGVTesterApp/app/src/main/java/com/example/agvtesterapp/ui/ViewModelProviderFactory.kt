package com.example.agvtesterapp.ui

import android.app.Application
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.agvtesterapp.repository.Repository

class ViewModelProviderFactory(val app: Application, val resultsRepository: Repository): ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return com.example.agvtesterapp.ui.ViewModel(app, resultsRepository) as T
    }
}