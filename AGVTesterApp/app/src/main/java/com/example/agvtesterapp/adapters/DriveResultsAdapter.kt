package com.example.agvtesterapp.adapters

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.AsyncListDiffer
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.RecyclerView
import com.example.agvtesterapp.R
import com.example.agvtesterapp.models.DetectedObject

/**
 * Class containing adapter for RecyclerView responsible
 * for displaying list of objects detected during drive
 */
class DriveResultsAdapter: RecyclerView.Adapter<DriveResultsAdapter.DetectedObjectViewHolder>() {

    /**
     * Inner class containing RecyclerView ViewHolder
     * @param itemView RecyclerView item view
     */
    inner class DetectedObjectViewHolder(itemView: View): RecyclerView.ViewHolder(itemView)

    /**
     * Defines how to compare items in detected objects list
     */
    private val differCallback = object : DiffUtil.ItemCallback<DetectedObject>() {

        override fun areItemsTheSame(oldItem: DetectedObject, newItem: DetectedObject): Boolean {
            return oldItem.name == newItem.name
        }

        override fun areContentsTheSame(oldItem: DetectedObject, newItem: DetectedObject): Boolean {
            return oldItem == newItem
        }
    }

    /**
     * Computes list differences in the background and updates RecyclerView
     * based on differCallback comparison rules
     */
    private val differ = AsyncListDiffer(this, differCallback)

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DetectedObjectViewHolder {
        return DetectedObjectViewHolder(
            LayoutInflater.from(parent.context).inflate(R.layout.item_detected_object, parent, false)
        )
    }

    override fun getItemCount(): Int {
        return differ.currentList.size
    }

    override fun onBindViewHolder(holder: DetectedObjectViewHolder, position: Int) {
        val detectedObject = differ.currentList[position]

        val objectName = holder.itemView.findViewById<TextView>(R.id.tvObjectName)
        val objectCount = holder.itemView.findViewById<TextView>(R.id.tvObjectCount)

        objectName.text = detectedObject.name
        objectCount.text = detectedObject.count.toString()
    }

    /**
     * Saves given list of detected objects in adapter's dedicated container
     * @param objectsList list of detected objects
     */
    fun addDetectedObjects(objectsList: List<DetectedObject>) {
        differ.submitList(objectsList)
    }
}