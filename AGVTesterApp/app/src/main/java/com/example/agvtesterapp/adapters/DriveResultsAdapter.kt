package com.example.agvtesterapp.adapters

import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.AsyncListDiffer
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.RecyclerView
import com.example.agvtesterapp.R
import com.example.agvtesterapp.models.DetectedObject
import com.example.agvtesterapp.websocket.WebSocketClient

class DriveResultsAdapter: RecyclerView.Adapter<DriveResultsAdapter.DetectedObjectViewHolder>() {

    inner class DetectedObjectViewHolder(itemView: View): RecyclerView.ViewHolder(itemView)

    private val differCallback = object : DiffUtil.ItemCallback<DetectedObject>() {

        override fun areItemsTheSame(oldItem: DetectedObject, newItem: DetectedObject): Boolean {
            return oldItem.name == newItem.name
        }

        override fun areContentsTheSame(oldItem: DetectedObject, newItem: DetectedObject): Boolean {
            return oldItem == newItem
        }
    }

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

    fun addDetectedObjects(objectsList: List<DetectedObject>) {
        objectsList.forEach { detectedObject ->
            Log.d(WebSocketClient.WEBSOCKET_TAG, "Adapter: ${detectedObject.name}, count: ${detectedObject.count}")
        }
        differ.submitList(objectsList)
    }
}