package com.cartfollow.tracker

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import kotlin.math.abs

/**
 * GPS course bearing + compass for brief steering assist when vision drops.
 */
class PhoneMotionAssist(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    private var lastLat = 0.0
    private var lastLon = 0.0
    private var hasLastFix = false
    var courseBearingDeg: Float? = null
        private set
    var compassAzimuthDeg: Float = 0f
        private set
    var speedMps: Float = 0f
        private set

    private var listening = false

    fun start() {
        if (listening) return
        rotationSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_UI)
        }
        listening = true
    }

    fun stop() {
        if (!listening) return
        sensorManager.unregisterListener(this)
        listening = false
        hasLastFix = false
        courseBearingDeg = null
    }

    fun onLocation(lat: Double, lon: Double, speed: Float) {
        speedMps = speed
        if (hasLastFix && speed >= MIN_SPEED_MPS) {
            val results = FloatArray(2)
            Location.distanceBetween(lastLat, lastLon, lat, lon, results)
            if (results[0] >= MIN_MOVE_M) {
                val bearing = bearingDeg(lastLat, lastLon, lat, lon)
                courseBearingDeg = bearing
            }
        }
        lastLat = lat
        lastLon = lon
        hasLastFix = true
    }

    fun canAssist(): Boolean {
        return courseBearingDeg != null && speedMps >= MIN_SPEED_MPS
    }

    /** Hold last steering during brief vision loss while cart is moving. */
    fun blendSteering(lastSteering: Int): Int {
        return (lastSteering * ASSIST_STEERING_SCALE).toInt()
            .coerceIn(-CartProtocol.MAX_STEERING, CartProtocol.MAX_STEERING)
    }

    fun formatHeadingLine(): String? {
        val brg = courseBearingDeg ?: return null
        return "HEADING brg=$brg spd=$speedMps az=$compassAzimuthDeg"
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        val rot = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(rot, event.values)
        val orient = FloatArray(3)
        SensorManager.getOrientation(rot, orient)
        compassAzimuthDeg = Math.toDegrees(orient[0].toDouble()).toFloat()
        if (compassAzimuthDeg < 0f) compassAzimuthDeg += 360f
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
    }

    private fun bearingDeg(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Float {
        val lat1Rad = Math.toRadians(lat1)
        val lat2Rad = Math.toRadians(lat2)
        val dLon = Math.toRadians(lon2 - lon1)
        val y = kotlin.math.sin(dLon) * kotlin.math.cos(lat2Rad)
        val x = kotlin.math.cos(lat1Rad) * kotlin.math.sin(lat2Rad) -
            kotlin.math.sin(lat1Rad) * kotlin.math.cos(lat2Rad) * kotlin.math.cos(dLon)
        var brg = Math.toDegrees(kotlin.math.atan2(y, x))
        if (brg < 0) brg += 360.0
        return brg.toFloat()
    }

    companion object {
        private const val MIN_SPEED_MPS = 0.4f
        private const val MIN_MOVE_M = 1.5f
        private const val ASSIST_STEERING_SCALE = 0.65f
    }
}
