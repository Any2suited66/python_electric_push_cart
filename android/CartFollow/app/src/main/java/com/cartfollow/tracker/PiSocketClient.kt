package com.cartfollow.tracker

import android.content.Context
import android.util.Log
import java.io.OutputStream
import java.net.Socket

/**
 * Write-only TCP client for the Pi bridge. We never read from the socket —
 * a background reader was dying on the Pi's CALIB ACK and falsely triggering
 * reconnect loops.
 */
class PiSocketClient(
    private val context: Context,
    val host: String,
    private val port: Int = CartProtocol.PI_PORT,
) {
    private var socket: Socket? = null
    private var output: OutputStream? = null
    private val writeLock = Any()

    fun connect(): Boolean {
        disconnect()
        var lastError: Exception? = null
        for (attempt in 1..CONNECT_ATTEMPTS) {
            try {
                val sock = CartNetwork.connectSocket(context, host, port, CONNECT_TIMEOUT_MS)
                    ?: throw java.net.ConnectException("no route to $host:$port")
                socket = sock
                output = sock.getOutputStream()
                return true
            } catch (e: Exception) {
                lastError = e
                Log.w(TAG, "Connect attempt $attempt/$CONNECT_ATTEMPTS failed: $host:$port", e)
                disconnect()
                if (attempt < CONNECT_ATTEMPTS) {
                    Thread.sleep(RETRY_DELAY_MS)
                }
            }
        }
        Log.w(TAG, "Connect gave up after $CONNECT_ATTEMPTS attempts", lastError)
        return false
    }

    fun send(packet: ByteArray): Boolean {
        return synchronized(writeLock) {
            val out = output ?: return false
            try {
                out.write(packet)
                out.flush()
                true
            } catch (e: Exception) {
                Log.w(TAG, "Send failed", e)
                false
            }
        }
    }

    fun sendLine(line: String): Boolean {
        return synchronized(writeLock) {
            val out = output ?: return false
            try {
                out.write((line.trim() + "\n").toByteArray(Charsets.US_ASCII))
                out.flush()
                true
            } catch (e: Exception) {
                Log.w(TAG, "Send line failed: $line", e)
                false
            }
        }
    }

    fun isConnected(): Boolean {
        val sock = socket ?: return false
        return sock.isConnected && !sock.isClosed
    }

    fun disconnect() {
        synchronized(writeLock) {
            try {
                output?.close()
            } catch (_: Exception) {
            }
            try {
                socket?.close()
            } catch (_: Exception) {
            }
            output = null
            socket = null
        }
    }

    companion object {
        private const val TAG = "PiSocketClient"
        private const val CONNECT_ATTEMPTS = 3
        private const val CONNECT_TIMEOUT_MS = 3000
        private const val RETRY_DELAY_MS = 400L
    }
}
