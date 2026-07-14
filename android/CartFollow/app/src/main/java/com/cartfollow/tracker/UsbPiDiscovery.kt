package com.cartfollow.tracker

import android.content.Context
import android.os.Build
import android.util.Log
import java.io.BufferedReader
import java.io.FileReader
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference

/**
 * Finds the Pi on the phone's USB-tether network (no Wi‑Fi required).
 *
 * Samsung often assigns 10.x.x.x on usb0/rndis0 (e.g. Pi at 10.141.179.71).
 */
object UsbPiDiscovery {
    private const val TAG = "UsbPiDiscovery"
    private const val PROBE_MS = 180
    private const val PARALLELISM = 32

    data class Result(
        val hosts: List<String>,
        val phoneUsbIps: List<String>,
        val method: String,
    )

    fun discover(context: Context): Result {
        val usbIfaces = usbInterfaces()
        val phoneIps = usbIfaces.map { it.ip }
        Log.i(TAG, "USB ifaces: ${usbIfaces.map { "${it.name}=${it.ip}/${it.prefix}" }}")

        if (usbIfaces.isEmpty()) {
            return Result(emptyList(), emptyList(), "no-usb")
        }

        val tetherClients = tetheredClientIps(context)
        if (tetherClients.isNotEmpty()) {
            Log.i(TAG, "Tethered clients: $tetherClients")
            return Result(tetherClients, phoneIps, "tether-clients")
        }

        val arp = arpOnUsb(usbIfaces.map { it.name }.toSet(), usbIfaces.map { it.prefix }.toSet())
        if (arp.isNotEmpty()) {
            Log.i(TAG, "USB ARP neighbors: $arp")
            return Result(arp, phoneIps, "arp")
        }

        // Full /24 candidates for each USB subnet (Pi DHCP can be any octet, e.g. .71).
        val hosts = linkedSetOf<String>()
        for (iface in usbIfaces) {
            for (o in 1..254) {
                if (o != iface.selfOctet) {
                    hosts.add("${iface.prefix}.$o")
                }
            }
        }
        Log.i(TAG, "USB subnet scan list size=${hosts.size}")
        return Result(hosts.toList(), phoneIps, "usb-subnet-scan")
    }

    /**
     * Probe [hosts] for open [port]; return first hit. Uses a thread pool so a /24
     * finishes in a few seconds instead of minutes.
     */
    fun findOpenPort(context: Context, hosts: List<String>, port: Int = CartProtocol.PI_PORT): String? {
        if (hosts.isEmpty()) return null
        if (hosts.size <= 8) {
            for (host in hosts) {
                if (probe(host, port)) return host
            }
            return null
        }

        val pool = Executors.newFixedThreadPool(PARALLELISM)
        val found = AtomicReference<String?>(null)
        val latch = CountDownLatch(hosts.size)
        try {
            for (host in hosts) {
                pool.execute {
                    try {
                        if (found.get() == null && probe(host, port)) {
                            found.compareAndSet(null, host)
                        }
                    } finally {
                        latch.countDown()
                    }
                }
            }
            // Don't wait forever — first hit wins; allow up to ~8s for a /24.
            latch.await(8, TimeUnit.SECONDS)
        } finally {
            pool.shutdownNow()
        }
        val hit = found.get()
        Log.i(TAG, if (hit != null) "Open $port at $hit" else "No open $port in ${hosts.size} hosts")
        return hit
    }

    private fun probe(host: String, port: Int): Boolean {
        return try {
            Socket().use { sock ->
                sock.tcpNoDelay = true
                sock.connect(InetSocketAddress(host, port), PROBE_MS)
                true
            }
        } catch (_: Exception) {
            false
        }
    }

    private data class UsbIface(val name: String, val ip: String, val prefix: String, val selfOctet: Int)

    private fun usbInterfaces(): List<UsbIface> {
        val out = mutableListOf<UsbIface>()
        try {
            for (iface in NetworkInterface.getNetworkInterfaces()) {
                if (!iface.isUp || iface.isLoopback) continue
                val name = iface.name.lowercase()
                if (!isUsbTetherName(name)) continue
                for (addr in iface.inetAddresses) {
                    if (addr !is Inet4Address || addr.isLoopbackAddress) continue
                    val ip = addr.hostAddress ?: continue
                    if (ip.startsWith("169.254.")) continue
                    val lastDot = ip.lastIndexOf('.')
                    if (lastDot <= 0) continue
                    val prefix = ip.substring(0, lastDot)
                    val octet = ip.substring(lastDot + 1).toIntOrNull() ?: continue
                    out.add(UsbIface(name, ip, prefix, octet))
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "USB iface enum failed", e)
        }
        return out
    }

    private fun isUsbTetherName(name: String): Boolean {
        return name.startsWith("rndis") ||
            name.startsWith("usb") ||
            name.startsWith("ncm") ||
            name.contains("tether") ||
            // Some Samsung builds expose tether as ap0 / wlan1-like; keep usb* primary.
            name == "ap0"
    }

    private fun arpOnUsb(ifaceNames: Set<String>, prefixes: Set<String>): List<String> {
        val found = linkedSetOf<String>()
        try {
            BufferedReader(FileReader("/proc/net/arp")).use { reader ->
                reader.readLine()
                var line = reader.readLine()
                while (line != null) {
                    val parts = line.trim().split(Regex("\\s+"))
                    if (parts.size >= 6) {
                        val ip = parts[0]
                        val flags = parts[2]
                        val mac = parts[3]
                        val device = parts[5].lowercase()
                        val lastDot = ip.lastIndexOf('.')
                        val prefix = if (lastDot > 0) ip.substring(0, lastDot) else ""
                        val reachable = flags != "0x0" && mac != "00:00:00:00:00:00"
                        val onUsb = device in ifaceNames ||
                            ifaceNames.any { device.startsWith(it.take(4)) } ||
                            prefix in prefixes
                        if (reachable && onUsb) found.add(ip)
                    }
                    line = reader.readLine()
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "ARP read failed", e)
        }
        return found.toList()
    }

    @Suppress("UNCHECKED_CAST")
    private fun tetheredClientIps(context: Context): List<String> {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.R) return emptyList()
        return try {
            val tmClass = Class.forName("android.net.TetheringManager")
            val tm = context.getSystemService(tmClass) ?: return emptyList()
            val ips = AtomicReference<List<String>>(emptyList())
            val latch = CountDownLatch(1)
            val callbackClass = Class.forName("android.net.TetheringManager\$TetheringEventCallback")
            val callback = java.lang.reflect.Proxy.newProxyInstance(
                callbackClass.classLoader,
                arrayOf(callbackClass),
            ) { _, method, args ->
                if (method.name == "onClientsChanged" && args != null && args.isNotEmpty()) {
                    val clients = args[0] as? Collection<*> ?: emptyList<Any>()
                    val found = mutableListOf<String>()
                    for (client in clients) {
                        if (client == null) continue
                        try {
                            val type = client.javaClass.getMethod("getTetheringType").invoke(client) as Int
                            if (type != 1) continue // TETHERING_USB
                            val addresses = client.javaClass.getMethod("getAddresses").invoke(client) as List<*>
                            for (info in addresses) {
                                if (info == null) continue
                                val link = info.javaClass.getMethod("getAddress").invoke(info)
                                val inet = link?.javaClass?.getMethod("getAddress")?.invoke(link)
                                val host = inet?.javaClass?.getMethod("getHostAddress")?.invoke(inet) as? String
                                if (host != null && !host.contains(":")) found.add(host)
                            }
                        } catch (_: Exception) {
                        }
                    }
                    ips.set(found.distinct())
                    latch.countDown()
                }
                null
            }
            tmClass.getMethod(
                "registerTetheringEventCallback",
                java.util.concurrent.Executor::class.java,
                callbackClass,
            ).invoke(tm, java.util.concurrent.Executor { it.run() }, callback)
            latch.await(800, TimeUnit.MILLISECONDS)
            try {
                tmClass.getMethod("unregisterTetheringEventCallback", callbackClass).invoke(tm, callback)
            } catch (_: Exception) {
            }
            ips.get()
        } catch (e: Exception) {
            Log.d(TAG, "TetheredClient API unavailable: ${e.message}")
            emptyList()
        }
    }
}
