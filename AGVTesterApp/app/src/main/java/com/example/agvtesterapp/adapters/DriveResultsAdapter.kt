package com.example.agvtesterapp.adapters

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.agvtesterapp.R
import com.example.agvtesterapp.models.DetectedObject

class DriveResultsAdapter: RecyclerView.Adapter<DriveResultsAdapter.DetectedObjectViewHolder>() {

    inner class DetectedObjectViewHolder(itemView: View): RecyclerView.ViewHolder(itemView)

    private val detectedObjectsList: ArrayList<DetectedObject> = ArrayList()

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DetectedObjectViewHolder {
        return DetectedObjectViewHolder(
            LayoutInflater.from(parent.context).inflate(R.layout.item_detected_object, parent, false)
        )
    }

    override fun getItemCount(): Int {
        return detectedObjectsList.size
    }

    override fun onBindViewHolder(holder: DetectedObjectViewHolder, position: Int) {
        val detectedObject = detectedObjectsList[position]

        val objectName = holder.itemView.findViewById<TextView>(R.id.tvObjectName)
        val objectTime = holder.itemView.findViewById<TextView>(R.id.tvObjectTime)

        objectName.text = detectedObject.name
        objectTime.text = detectedObject.time
    }

    fun addObject(obj: DetectedObject) {
        detectedObjectsList.add(obj)
    }

    fun getObjets(): ArrayList<DetectedObject> = detectedObjectsList

    fun clearObjects() {
        detectedObjectsList.clear()
    }
}