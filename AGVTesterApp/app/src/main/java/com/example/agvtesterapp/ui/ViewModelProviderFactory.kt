package com.example.agvtesterapp.ui

import android.app.Application
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.agvtesterapp.repository.Repository

/**
 * View model factory - creates an instance of the [ViewModel] class
 * @param app reference to the [Application] class that owns this view model
 * @param resultsRepository reference to the [Repository] instance
 */
class ViewModelProviderFactory(val app: Application, val resultsRepository: Repository): ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return com.example.agvtesterapp.ui.ViewModel(app, resultsRepository) as T
    }
}