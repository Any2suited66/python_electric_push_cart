package com.cartfollow

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.util.Log
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.cartfollow.databinding.ActivityMainBinding
import com.cartfollow.tracker.CartNetwork
import com.cartfollow.tracker.CartProtocol
import com.cartfollow.tracker.NetworkScanner
import com.cartfollow.tracker.PiSocketClient
import com.cartfollow.tracker.UsbPiDiscovery
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * UI for cart follow. Camera, GPS, and Pi streaming run in [CartFollowService]
 * so tracking continues when the screen turns off.
 */
class MainActivity : AppCompatActivity() {
    private lateinit var binding: ActivityMainBinding
    private val scanning = AtomicBoolean(false)
    private val scanExecutor = Executors.newSingleThreadExecutor()
    private var followService: CartFollowService? = null
    private var serviceBound = false
    private var sessionActive = false

    companion object {
        private const val TAG = "CartFollowUI"
    }

    private val serviceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, binder: IBinder?) {
            val service = (binder as CartFollowService.LocalBinder).getService()
            followService = service
            serviceBound = true
            service.setStatusListener { status -> runOnUiThread { applyStatus(status) } }
            service.attachPreview(binding.previewView.surfaceProvider)
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            followService = null
            serviceBound = false
        }
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { grants ->
        val needed = mutableListOf<String>()
        if (grants[Manifest.permission.CAMERA] != true) {
            toast("Camera permission required")
        }
        if (grants[Manifest.permission.ACCESS_FINE_LOCATION] != true) {
            toast("Location permission needed for yardages")
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU &&
            grants[Manifest.permission.POST_NOTIFICATIONS] != true
        ) {
            toast("Notification permission keeps tracking alive with screen off")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.connectButton.setOnClickListener {
            if (sessionActive) disconnectPi() else scanAndConnect()
        }

        binding.calibrateButton.setOnClickListener { startCalibration() }

        updateCalibButton(connected = false, calibrating = false, calibrated = false)

        ensurePermissions()
    }

    override fun onStart() {
        super.onStart()
        bindService(
            Intent(this, CartFollowService::class.java),
            serviceConnection,
            Context.BIND_AUTO_CREATE,
        )
    }

    override fun onResume() {
        super.onResume()
        followService?.attachPreview(binding.previewView.surfaceProvider)
    }

    override fun onPause() {
        followService?.detachPreview()
        super.onPause()
    }

    override fun onStop() {
        if (serviceBound) {
            followService?.setStatusListener(null)
            unbindService(serviceConnection)
            serviceBound = false
            followService = null
        }
        super.onStop()
    }

    private fun scanAndConnect() {
        if (scanning.get()) return
        scanning.set(true)
        binding.connectButton.isEnabled = false
        binding.connectButton.text = getString(R.string.scanning)
        setStatus("Finding Pi on USB…")

        val manualRaw = binding.piIpInput.text.toString().trim()
        val manual = NetworkScanner.extractIpv4(manualRaw) ?: manualRaw
        if (manualRaw.isNotEmpty() && manual != manualRaw && manual.isNotEmpty()) {
            binding.piIpInput.setText(manual)
        }
        val hole = binding.holeInput.text.toString().trim().toIntOrNull()?.coerceIn(1, 18) ?: 1
        val appCtx = applicationContext
        val port = CartProtocol.PI_PORT

        scanExecutor.execute {
            try {
                val phoneUsb = CartNetwork.localUsbIpv4s()
                // Typed IP on a different /24 than phone USB = stale Pi lease; ignore and scan.
                val useManual = manual.isNotEmpty() && (
                    phoneUsb.isEmpty() || phoneUsb.any { sameSlash24(it, manual) }
                )
                if (manual.isNotEmpty() && !useManual) {
                    Log.w(TAG, "Ignoring stale Pi IP $manual (phone USB ${phoneUsb.joinToString()})")
                    runOnUiThread {
                        setStatus(
                            "Stale Pi IP $manual — scanning phone USB ${phoneUsb.first()}…",
                        )
                    }
                }

                if (useManual) {
                    runOnUiThread { setStatus("Connecting to $manual…") }
                    if (connectToHost(appCtx, manual, hole)) {
                        return@execute
                    }
                    // Wrong/stale IP: fall through to USB scan instead of hard-failing.
                    runOnUiThread {
                        setStatus("Could not reach $manual — scanning USB tether…")
                    }
                }

                // Golf-course path: USB tether only (no Wi‑Fi needed).
                runOnUiThread { setStatus("Scanning USB tether for Pi…") }
                val discovery = UsbPiDiscovery.discover(appCtx)
                if (discovery.phoneUsbIps.isEmpty()) {
                    finishConnectAttempt(
                        null,
                        "No USB network on phone. Enable USB tethering while plugged into the Pi.",
                    )
                    return@execute
                }

                runOnUiThread {
                    setStatus(
                        "USB ${discovery.phoneUsbIps.first()} — finding Pi (${discovery.method})…",
                    )
                }
                val host = UsbPiDiscovery.findOpenPort(appCtx, discovery.hosts, port)
                if (host != null) {
                    runOnUiThread {
                        binding.piIpInput.setText(host)
                        setStatus("Connecting to $host (USB)…")
                    }
                    if (connectToHost(appCtx, host, hole)) {
                        return@execute
                    }
                }

                finishConnectAttempt(
                    null,
                    "No Pi on USB :$port. phone=${discovery.phoneUsbIps.joinToString()} " +
                        "(${discovery.method}, ${discovery.hosts.size} hosts). " +
                        "On Pi: ip -4 addr show usb0 && sudo dhclient -v usb0 — " +
                        "Pi must be on the same 10.x subnet as the phone, and " +
                        "hoverboard_minimal.py must be listening on :$port.",
                )
            } catch (e: Exception) {
                Log.e(TAG, "Connect scan failed", e)
                finishConnectAttempt(null, "Connect error: ${e.message ?: e.javaClass.simpleName}")
            }
        }
    }

    private fun sameSlash24(a: String, b: String): Boolean {
        val da = a.lastIndexOf('.')
        val db = b.lastIndexOf('.')
        if (da <= 0 || db <= 0) return false
        return a.substring(0, da) == b.substring(0, db)
    }

    private fun connectToHost(appCtx: android.content.Context, host: String, hole: Int): Boolean {
        val probe = PiSocketClient(appCtx, host)
        if (!probe.connect()) {
            return false
        }
        probe.disconnect()
        runOnUiThread {
            sessionActive = true
            CartFollowService.start(appCtx, host, hole)
            finishConnectAttempt(host, getString(R.string.status_connected))
        }
        return true
    }

    private fun disconnectPi() {
        sessionActive = false
        CartFollowService.stop(applicationContext)
        binding.connectButton.text = getString(R.string.scan_connect)
        binding.gpsText.visibility = View.GONE
        binding.calibOverlay.visibility = View.GONE
        updateCalibButton(connected = false, calibrating = false, calibrated = false)
        setStatus(getString(R.string.status_idle))
    }

    private fun startCalibration() {
        if (!sessionActive) {
            toast(getString(R.string.calibrate_connect_first))
            return
        }
        followService?.startCalibration() ?: CartFollowService.calibrate(applicationContext)
    }

    private fun finishConnectAttempt(connectedHost: String?, message: String) {
        runOnUiThread {
            scanning.set(false)
            binding.connectButton.isEnabled = true
            if (connectedHost != null) {
                binding.piIpInput.setText(connectedHost)
                binding.connectButton.text = getString(R.string.disconnect)
                // Calib button state comes from service status (may already be calibrated).
            } else {
                binding.connectButton.text = getString(R.string.scan_connect)
                updateCalibButton(connected = false, calibrating = false, calibrated = false)
            }
            setStatus(message)
        }
    }

    private fun applyStatus(status: CartFollowService.ServiceStatus) {
        if (status.connected) {
            sessionActive = true
            binding.connectButton.text = getString(R.string.disconnect)
        } else {
            sessionActive = false
            if (!scanning.get()) {
                binding.connectButton.text = getString(R.string.scan_connect)
            }
        }
        updateCalibButton(status.connected, status.calibrating, status.calibrated)

        if (status.calibrating) {
            binding.calibOverlay.visibility = View.VISIBLE
            if (status.calibTitle.isNotEmpty()) {
                binding.calibTitle.text = status.calibTitle
            }
            if (status.calibCountdown > 0) {
                binding.calibCountdown.text = status.calibCountdown.toString()
            }
            binding.calibSubtitle.text = status.message
        } else {
            binding.calibOverlay.visibility = View.GONE
        }

        status.gpsAccuracyM?.let { acc ->
            binding.gpsText.visibility = View.VISIBLE
            binding.gpsText.text = "GPS ±${acc}m"
        }
        val track = status.track
        if (track != null && status.connected) {
            val cal = when {
                status.calibrating -> "cal"
                status.calibrated -> "ok"
                else -> "—"
            }
            val assist = if (status.visionAssist) " gps" else ""
            val color = if (!track.colorMatch) " !color" else ""
            setStatus(
                "${status.message} | Pi OK [$cal] det=${track.detected}$assist$color " +
                    "S=${track.steering} T=${track.throttle}",
            )
        } else {
            setStatus(status.message)
        }
    }

    private fun updateCalibButton(connected: Boolean, calibrating: Boolean, calibrated: Boolean) {
        if (!connected) {
            binding.calibrateButton.visibility = View.GONE
            return
        }
        binding.calibrateButton.visibility = View.VISIBLE
        binding.calibrateButton.isEnabled = !calibrating
        binding.calibrateButton.text = when {
            calibrating -> getString(R.string.calibrating)
            calibrated -> getString(R.string.recalibrate)
            else -> getString(R.string.calibrate)
        }
    }

    private fun ensurePermissions() {
        val needed = mutableListOf<String>()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED
        ) {
            needed.add(Manifest.permission.CAMERA)
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) {
            needed.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS)
            != PackageManager.PERMISSION_GRANTED
        ) {
            needed.add(Manifest.permission.POST_NOTIFICATIONS)
        }
        if (needed.isNotEmpty()) {
            permissionLauncher.launch(needed.toTypedArray())
        }
    }

    private fun setStatus(text: String) {
        binding.statusText.text = text
    }

    private fun toast(msg: String) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
    }

    override fun onDestroy() {
        scanExecutor.shutdownNow()
        super.onDestroy()
    }
}
