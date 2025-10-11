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

    private val detectedObjectListK: ArrayList<DetectedObject> = ArrayList()

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DetectedObjectViewHolder {
        return DetectedObjectViewHolder(
            LayoutInflater.from(parent.context).inflate(R.layout.item_detected_object, parent, false)
        )
    }

    override fun getItemCount(): Int {
        return detectedObjectListK.size
    }

    override fun onBindViewHolder(holder: DetectedObjectViewHolder, position: Int) {
        val detectedObject = detectedObjectListK[position]

        val objectName = holder.itemView.findViewById<TextView>(R.id.tvObjectName)
        val objectCount = holder.itemView.findViewById<TextView>(R.id.tvObjectCount)

        objectName.text = detectedObject.name
        objectCount.text = detectedObject.count.toString()
    }

    fun addObject(obj: DetectedObject) {
        detectedObjectListK.add(obj)
    }

    fun addObjectsList(objectsList: ArrayList<DetectedObject>) {
        detectedObjectListK.addAll(objectsList)
    }

    fun replaceObjectsList(objectsList: ArrayList<DetectedObject>) {
        clearObjects()
        addObjectsList(objectsList)
    }

    fun getObjets(): ArrayList<DetectedObject> = detectedObjectListK

    fun clearObjects() {
        detectedObjectListK.clear()
    }
}