package com.cartfollow.tracker

import android.content.Context
import android.util.Log
import java.io.BufferedReader
import java.io.FileReader
import java.net.Inet4Address
import java.net.NetworkInterface

/**
 * Finds the Pi on USB tether (and nearby LANs) without scanning 250+ IPs for minutes.
 *
 * Order:
 * 1. ARP neighbors on tether-like interfaces (live USB clients)
 * 2. A short list of common gateway/client octets on tether subnets
 * 3. Optional broader sweep only of the best tether subnet (still capped)
 */
object NetworkScanner {
    private const val TAG = "NetworkScanner"
    private const val PROBE_TIMEOUT_MS = 350
    private const val MAX_SUBNET_SWEEP = 40

    private data class Subnet(val prefix: String, val selfOctet: Int, val priority: Int, val iface: String)

    fun localTetherHints(context: Context): List<String> {
        return candidateSubnets().map {
            "${it.iface} ${it.prefix}.${it.selfOctet}"
        }
    }

    /**
     * Ordered Pi IP candidates (most likely first). Caps size so Connect finishes quickly.
     */
    fun piHostCandidates(context: Context): List<String> {
        val subnets = candidateSubnets()
        if (subnets.isEmpty()) {
            Log.w(TAG, "No IPv4 interface found — is USB tethering on?")
            return emptyList()
        }

        val hosts = linkedSetOf<String>()
        val arp = arpNeighbors(subnets.map { it.prefix }.toSet())
        Log.i(TAG, "ARP neighbors: $arp")
        hosts.addAll(arp)

        // Known cart Pi default on this project.
        hosts.add("192.168.4.221")

        val quickOctets = listOf(1, 2, 10, 15, 20, 42, 50, 100, 129, 150, 178, 200, 221)
        for (subnet in subnets) {
            for (octet in quickOctets) {
                if (octet != subnet.selfOctet) {
                    hosts.add("${subnet.prefix}.$octet")
                }
            }
        }

        // Limited sweep of the highest-priority tether subnet only.
        val best = subnets.first()
        var added = 0
        for (octet in 1..254) {
            if (octet == best.selfOctet) continue
            if (hosts.add("${best.prefix}.$octet")) {
                added++
                if (added >= MAX_SUBNET_SWEEP) break
            }
        }

        Log.i(
            TAG,
            "Pi candidates (${hosts.size}): tether=${subnets.map { "${it.iface}/${it.prefix}.${it.selfOctet}" }}",
        )
        return hosts.toList()
    }

    /**
     * Pull an IPv4 from pasted text (e.g. `hostname -I` output).
     * When several addresses are present, prefer the USB-tether one (10.x / rndis DHCP),
     * not the Pi's Wi‑Fi address (often 192.168.4.x in this project).
     */
    fun extractIpv4(raw: String): String? {
        val ips = Regex("""\b(\d{1,3}(?:\.\d{1,3}){3})\b""")
            .findAll(raw.trim())
            .map { it.groupValues[1] }
            .filter { ip ->
                val parts = ip.split('.').mapNotNull { it.toIntOrNull() }
                parts.size == 4 && parts.all { it in 0..255 }
            }
            .toList()
        if (ips.isEmpty()) return null
        if (ips.size == 1) return ips[0]
        return ips.minByOrNull { tetherIpPriority(it) }
    }

    /** Lower = better match for Pi on phone USB tether. */
    private fun tetherIpPriority(ip: String): Int = when {
        ip.startsWith("10.") -> 0
        ip.startsWith("192.168.42.") || ip.startsWith("192.168.43.") -> 1
        ip.startsWith("192.168.137.") -> 2
        ip.startsWith("192.168.4.") -> 9 // Pi wlan on home LAN — wrong for USB-only
        ip.startsWith("192.168.") -> 5
        else -> 6
    }

    /**
     * Fast single-shot TCP probe. Prefer this over [PiSocketClient.connect] while scanning.
     */
    fun probePort(context: Context, host: String, port: Int = CartProtocol.PI_PORT): Boolean {
        // Same policy as CartNetwork: tether IPs via default route first.
        return CartNetwork.connectSocket(context, host, port, PROBE_TIMEOUT_MS)?.let { sock ->
            try {
                sock.close()
            } catch (_: Exception) {
            }
            true
        } ?: false
    }

    private fun candidateSubnets(): List<Subnet> {
        val subnets = mutableListOf<Subnet>()
        try {
            for (iface in NetworkInterface.getNetworkInterfaces()) {
                if (!iface.isUp || iface.isLoopback) continue
                val name = iface.name.lowercase()
                val tetherLike = name.startsWith("rndis") ||
                    name.startsWith("usb") ||
                    name.startsWith("ncm") ||
                    name.contains("tether") ||
                    name.startsWith("ap")

                for (addr in iface.inetAddresses) {
                    if (addr !is Inet4Address || addr.isLoopbackAddress) continue
                    val ip = addr.hostAddress ?: continue
                    // Skip link-local
                    if (ip.startsWith("169.254.")) continue
                    val lastDot = ip.lastIndexOf('.')
                    if (lastDot <= 0) continue
                    val prefix = ip.substring(0, lastDot)
                    val selfOctet = ip.substring(lastDot + 1).toIntOrNull() ?: continue

                    val priority = when {
                        tetherLike && prefix == "192.168.4" -> 0
                        prefix == "192.168.4" -> 1
                        tetherLike && (prefix == "192.168.42" || prefix == "192.168.43") -> 2
                        tetherLike -> 3
                        prefix == "192.168.42" || prefix == "192.168.43" -> 4
                        prefix == "192.168.137" -> 5
                        name.startsWith("wlan") -> 8
                        prefix.startsWith("10.") -> 9
                        else -> 6
                    }
                    subnets.add(Subnet(prefix, selfOctet, priority, name))
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Interface enumeration failed", e)
        }

        return subnets.distinctBy { it.prefix }.sortedBy { it.priority }
    }

    /** Live neighbors from /proc/net/arp (no root needed on most devices). */
    private fun arpNeighbors(prefixes: Set<String>): List<String> {
        val found = mutableListOf<String>()
        try {
            BufferedReader(FileReader("/proc/net/arp")).use { reader ->
                reader.readLine() // header
                var line = reader.readLine()
                while (line != null) {
                    val parts = line.trim().split(Regex("\\s+"))
                    if (parts.size >= 4) {
                        val ip = parts[0]
                        val flags = parts[2]
                        val mac = parts[3]
                        val lastDot = ip.lastIndexOf('.')
                        val prefix = if (lastDot > 0) ip.substring(0, lastDot) else ""
                        val reachable = flags != "0x0" && mac != "00:00:00:00:00:00"
                        if (reachable && (prefixes.isEmpty() || prefix in prefixes)) {
                            found.add(ip)
                        }
                    }
                    line = reader.readLine()
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "ARP read failed", e)
        }
        return found
    }
}
