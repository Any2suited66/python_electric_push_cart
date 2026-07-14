package com.cartfollow.tracker

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.util.Log
import java.net.InetSocketAddress
import java.net.Socket

object CartNetwork {
    private const val TAG = "CartNetwork"

    fun preferredNetworks(context: Context): List<Network> {
        val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        val usb = mutableListOf<Network>()
        val wifi = mutableListOf<Network>()
        val other = mutableListOf<Network>()
        for (net in cm.allNetworks) {
            val caps = cm.getNetworkCapabilities(net) ?: continue
            when {
                caps.hasTransport(NetworkCapabilities.TRANSPORT_USB) ||
                    caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET) ->
                    usb.add(net)
                caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ->
                    wifi.add(net)
                caps.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) ->
                    continue // skip cellular for cart link
                else -> other.add(net)
            }
        }
        return usb + wifi + other
    }

    fun isLikelyTetherHost(host: String): Boolean {
        return host.startsWith("192.168.42.") ||
            host.startsWith("192.168.43.") ||
            host.startsWith("192.168.137.") ||
            host.startsWith("192.168.4.") ||
            host.startsWith("10.") // Samsung USB tether often uses 10.x
    }

    fun connectSocket(context: Context, host: String, port: Int, timeoutMs: Int): Socket? {
        // USB-tether / 10.x: default route first — binding Wi‑Fi black-holes usb0 peers.
        if (isLikelyTetherHost(host)) {
            openDefault(host, port, timeoutMs)?.let { return it }
        }

        for (net in preferredNetworks(context)) {
            openOnNetwork(net, host, port, timeoutMs)?.let { return it }
        }
        return openDefault(host, port, timeoutMs)
    }

    private fun openOnNetwork(net: Network, host: String, port: Int, timeoutMs: Int): Socket? {
        val sock = Socket()
        return try {
            net.bindSocket(sock)
            configureSocket(sock)
            sock.connect(InetSocketAddress(host, port), timeoutMs)
            Log.i(TAG, "Connected to $host:$port via $net")
            sock
        } catch (e: Exception) {
            Log.d(TAG, "Connect via $net to $host:$port failed: ${e.message}")
            try {
                sock.close()
            } catch (_: Exception) {
            }
            null
        }
    }

    private fun openDefault(host: String, port: Int, timeoutMs: Int): Socket? {
        return try {
            val sock = Socket()
            configureSocket(sock)
            sock.connect(InetSocketAddress(host, port), timeoutMs)
            Log.i(TAG, "Connected to $host:$port via default route")
            sock
        } catch (e: Exception) {
            Log.w(TAG, "Default connect failed for $host:$port: ${e.message}")
            null
        }
    }

    private fun configureSocket(sock: Socket) {
        sock.tcpNoDelay = true
        sock.keepAlive = true
        sock.soTimeout = 0
    }
}
