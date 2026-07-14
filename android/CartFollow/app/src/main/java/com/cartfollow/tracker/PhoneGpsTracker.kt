package com.cartfollow.tracker

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat

/**
 * Streams phone GPS fixes to the Pi over ASCII lines.
 * Replaces a USB GPS dongle on the cart for yardage + summon logging.
 */
class PhoneGpsTracker(
    private val context: Context,
    private val onFix: (lat: Double, lon: Double, accuracyM: Float, speedMps: Float) -> Unit,
) {
    private val locationManager =
        context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private var running = false
    private var gpsLooper: Looper? = null

    private val listener = object : LocationListener {
        override fun onLocationChanged(location: Location) {
            onFix(
                location.latitude,
                location.longitude,
                location.accuracy,
                location.speed.coerceAtLeast(0f),
            )
        }

        @Deprecated("Deprecated in Java")
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {
        }

        override fun onProviderEnabled(provider: String) {
        }

        override fun onProviderDisabled(provider: String) {
        }
    }

    fun hasPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_FINE_LOCATION,
        ) == PackageManager.PERMISSION_GRANTED
    }

    fun start() {
        if (running || !hasPermission()) return
        running = true
        val thread = HandlerThread("gps-updates").apply { start() }
        gpsLooper = thread.looper
        val looper = thread.looper
        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    UPDATE_INTERVAL_MS,
                    MIN_DISTANCE_M,
                    listener,
                    looper,
                )
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    UPDATE_INTERVAL_MS,
                    MIN_DISTANCE_M * 2,
                    listener,
                    looper,
                )
            }
            locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)?.let {
                listener.onLocationChanged(it)
            }
        } catch (e: SecurityException) {
            Log.w(TAG, "Location permission missing", e)
            running = false
        }
    }

    fun stop() {
        if (!running) return
        running = false
        try {
            locationManager.removeUpdates(listener)
        } catch (_: Exception) {
        }
        gpsLooper?.let { (it.thread as? HandlerThread)?.quitSafely() }
        gpsLooper = null
    }

    fun formatGpsLine(lat: Double, lon: Double, accuracyM: Float, speedMps: Float): String {
        return "GPS lat=$lat lon=$lon acc=$accuracyM spd=$speedMps"
    }

    companion object {
        private const val TAG = "PhoneGpsTracker"
        private const val UPDATE_INTERVAL_MS = 1000L
        private const val MIN_DISTANCE_M = 1f
    }
}
