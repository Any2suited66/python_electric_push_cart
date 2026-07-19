package com.cartfollow.tracker

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.util.Log
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket

/**
 * Opens TCP sockets to the Pi, preferring the USB-tether path.
 *
 * When the phone is the USB-tether *host*, Android often has no ConnectivityManager
 * [Network] for that interface. Binding the socket to the phone's local USB IPv4
 * (e.g. 10.141.179.1) forces traffic onto usb0/rndis0 so Wi‑Fi cannot black-hole it.
 */
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
        val triedLocal = mutableSetOf<String>()

        // 1) Force source address onto phone USB interfaces that share the Pi subnet.
        for (local in localUsbIpv4s()) {
            if (!sameSlash24(local, host)) continue
            triedLocal.add(local)
            openBoundLocal(local, host, port, timeoutMs)?.let { return it }
        }

        // 2) Any ConnectivityManager Network whose LinkProperties covers this host.
        val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        for (net in networksForHost(cm, host)) {
            openOnNetwork(net, host, port, timeoutMs)?.let { return it }
        }

        // 3) USB/ethernet networks first, then others (still try Wi‑Fi for LAN Pi IPs).
        for (net in preferredNetworks(context)) {
            openOnNetwork(net, host, port, timeoutMs)?.let { return it }
        }

        // 4) Remaining local USB IPs (different subnet guesses / weird Samsung ranges).
        for (local in localUsbIpv4s()) {
            if (local in triedLocal) continue
            openBoundLocal(local, host, port, timeoutMs)?.let { return it }
        }

        // 5) Default route (works when Wi‑Fi is off and kernel routes to usb0).
        return openDefault(host, port, timeoutMs)
    }

    /**
     * Fast scan probe: one attempt bound to the phone USB address on the same /24.
     * Avoids stacking multi-path timeouts that made /24 scans miss the Pi.
     */
    fun probeUsbPort(host: String, port: Int, timeoutMs: Int): Boolean {
        var triedUsb = false
        for (local in localUsbIpv4s()) {
            if (!sameSlash24(local, host)) continue
            triedUsb = true
            openBoundLocal(local, host, port, timeoutMs)?.let { sock ->
                try {
                    sock.close()
                } catch (_: Exception) {
                }
                return true
            }
        }
        if (triedUsb) return false
        // No USB iface on this /24 — fall back to default route once.
        return openDefault(host, port, timeoutMs)?.let { sock ->
            try {
                sock.close()
            } catch (_: Exception) {
            }
            true
        } ?: false
    }

    /** Phone IPv4 addresses on usb/rndis/ncm/tether interfaces. */
    fun localUsbIpv4s(): List<String> {
        val out = linkedSetOf<String>()
        try {
            for (iface in NetworkInterface.getNetworkInterfaces()) {
                if (!iface.isUp || iface.isLoopback) continue
                val name = iface.name.lowercase()
                if (!isUsbTetherName(name)) continue
                for (addr in iface.inetAddresses) {
                    if (addr !is Inet4Address || addr.isLoopbackAddress) continue
                    val ip = addr.hostAddress ?: continue
                    if (ip.startsWith("169.254.")) continue
                    out.add(ip)
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "USB iface enum failed", e)
        }
        return out.toList()
    }

    private fun isUsbTetherName(name: String): Boolean {
        return name.startsWith("rndis") ||
            name.startsWith("usb") ||
            name.startsWith("ncm") ||
            name.contains("tether") ||
            name == "ap0"
    }

    private fun sameSlash24(a: String, b: String): Boolean {
        val da = a.lastIndexOf('.')
        val db = b.lastIndexOf('.')
        if (da <= 0 || db <= 0) return false
        return a.substring(0, da) == b.substring(0, db)
    }

    private fun networksForHost(cm: ConnectivityManager, host: String): List<Network> {
        val matched = mutableListOf<Network>()
        for (net in cm.allNetworks) {
            val lp = cm.getLinkProperties(net) ?: continue
            for (link in lp.linkAddresses) {
                val addr = link.address
                if (addr !is Inet4Address) continue
                val local = addr.hostAddress ?: continue
                if (sameSlash24(local, host)) {
                    matched.add(net)
                    break
                }
            }
        }
        return matched
    }

    private fun openBoundLocal(localIp: String, host: String, port: Int, timeoutMs: Int): Socket? {
        val sock = Socket()
        return try {
            configureSocket(sock)
            sock.bind(InetSocketAddress(localIp, 0))
            sock.connect(InetSocketAddress(host, port), timeoutMs)
            Log.i(TAG, "Connected to $host:$port via local bind $localIp")
            sock
        } catch (e: Exception) {
            Log.d(TAG, "Local-bind $localIp → $host:$port failed: ${e.message}")
            try {
                sock.close()
            } catch (_: Exception) {
            }
            null
        }
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
