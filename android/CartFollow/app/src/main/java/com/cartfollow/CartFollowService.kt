package com.cartfollow

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.IBinder
import android.util.Log
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleService
import com.cartfollow.tracker.BodyCalibrator
import com.cartfollow.tracker.CalibrationTone
import com.cartfollow.tracker.CalibStore
import com.cartfollow.tracker.CartProtocol
import com.cartfollow.tracker.PersonTracker
import com.cartfollow.tracker.PhoneGpsTracker
import com.cartfollow.tracker.PhoneMotionAssist
import com.cartfollow.tracker.PiSocketClient
import com.cartfollow.tracker.TrackResult
import com.google.mlkit.vision.common.InputImage
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference

/**
 * Keeps camera tracking, GPS, and Pi streaming alive when the screen is off.
 * Requires a foreground notification (Android policy).
 */
class CartFollowService : LifecycleService() {
    private val binder = LocalBinder()
    private val tracker = PersonTracker()
    private lateinit var gpsTracker: PhoneGpsTracker
    private lateinit var motionAssist: PhoneMotionAssist
    private var piClient: PiSocketClient? = null
    private val piConnected = AtomicBoolean(false)
    private val streaming = AtomicBoolean(false)
    private val calibrating = AtomicBoolean(false)
    private var connectedHost: String? = null
    private var holeNumber = 1
    private val worker = Executors.newSingleThreadExecutor()
    private val sendExecutor = Executors.newSingleThreadExecutor()
    private val cameraExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    private val lastTrack = AtomicReference(TrackResult(false, 0f, 0, 0))
    private val frameWidth = AtomicInteger(0)
    private var cameraProvider: ProcessCameraProvider? = null
    private var analysis: ImageAnalysis? = null
    private var preview: Preview? = null
    private var statusListener: ((ServiceStatus) -> Unit)? = null

    data class ServiceStatus(
        val connected: Boolean,
        val message: String,
        val gpsAccuracyM: Int? = null,
        val track: TrackResult? = null,
        val calibrating: Boolean = false,
        val calibrated: Boolean = false,
        val calibCountdown: Int = 0,
        val calibTitle: String = "",
        val visionAssist: Boolean = false,
    )

    inner class LocalBinder : Binder() {
        fun getService(): CartFollowService = this@CartFollowService
    }

    override fun onCreate() {
        super.onCreate()
        motionAssist = PhoneMotionAssist(applicationContext)
        CalibStore.loadInto(applicationContext, tracker.calibrator)
        gpsTracker = PhoneGpsTracker(applicationContext) { lat, lon, acc, spd ->
            motionAssist.onLocation(lat, lon, spd)
            notifyStatus(buildStatus(gpsAccuracyM = acc.toInt()))
            if (piConnected.get()) {
                sendLineOrDrop(gpsTracker.formatGpsLine(lat, lon, acc, spd))
                motionAssist.formatHeadingLine()?.let { sendLineOrDrop(it) }
            }
        }
        createNotificationChannel()
    }

    override fun onBind(intent: Intent): IBinder {
        super.onBind(intent)
        return binder
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        super.onStartCommand(intent, flags, startId)
        when (intent?.action) {
            ACTION_START -> {
                val host = intent.getStringExtra(EXTRA_HOST) ?: return START_NOT_STICKY
                holeNumber = intent.getIntExtra(EXTRA_HOLE, 1).coerceIn(1, 18)
                startSession(host)
            }
            ACTION_CALIBRATE -> startCalibration()
            ACTION_STOP -> stopSession()
        }
        return START_STICKY
    }

    fun setStatusListener(listener: ((ServiceStatus) -> Unit)?) {
        statusListener = listener
        listener?.invoke(buildStatus())
    }

    fun attachPreview(surfaceProvider: Preview.SurfaceProvider) {
        val provider = cameraProvider ?: return
        preview = Preview.Builder().build().also {
            it.setSurfaceProvider(surfaceProvider)
        }
        try {
            provider.unbindAll()
            val cases = mutableListOf(analysis!!, preview!!)
            provider.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, *cases.toTypedArray())
        } catch (e: Exception) {
            Log.w(TAG, "Preview attach failed", e)
        }
    }

    fun detachPreview() {
        preview = null
        val provider = cameraProvider ?: return
        val analysisUseCase = analysis ?: return
        try {
            provider.unbindAll()
            provider.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, analysisUseCase)
        } catch (e: Exception) {
            Log.w(TAG, "Preview detach failed", e)
        }
    }

    fun startCalibration() {
        if (!piConnected.get() || calibrating.get()) return
        worker.execute { runCalibration() }
    }

    private fun startSession(host: String) {
        if (piConnected.get()) return
        startForeground(NOTIFICATION_ID, buildNotification("Connecting to $host…"))
        worker.execute {
            val client = PiSocketClient(applicationContext, host)
            if (!client.connect()) {
                notifyStatus(
                    buildStatus(
                        connected = false,
                        message = "Connect failed — calibration kept, try again",
                    ),
                )
                // Keep service alive if we already have camera/calib from a prior session.
                if (cameraProvider == null) {
                    stopSelf()
                }
                return@execute
            }
            piClient = client
            piConnected.set(true)
            connectedHost = host
            if (gpsTracker.hasPermission()) {
                gpsTracker.start()
                motionAssist.start()
            }
            startCameraAnalysis()
            piClient?.sendLine("HOLE $holeNumber")
            if (tracker.calibrator.isCalibrated) {
                sendCalibOkLine()
            }
            beginStreaming()
            val msg = if (tracker.calibrator.isCalibrated) {
                getString(R.string.status_reconnected)
            } else {
                getString(R.string.status_connected)
            }
            notifyStatus(buildStatus(message = msg))
            updateNotification(msg)
        }
    }

    private fun stopSession() {
        streaming.set(false)
        calibrating.set(false)
        piConnected.set(false)
        gpsTracker.stop()
        motionAssist.stop()
        // Keep body/color calibration so reconnect does not require re-calib.
        tracker.resetSmoothing()
        piClient?.disconnect()
        piClient = null
        connectedHost = null
        cameraProvider?.unbindAll()
        CalibrationTone.release()
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    /** TCP write failed or USB dropped — keep calibration, prompt reconnect. */
    private fun handleLinkLost(reason: String = "Pi disconnected") {
        if (!piConnected.getAndSet(false)) return
        streaming.set(false)
        calibrating.set(false)
        try {
            piClient?.disconnect()
        } catch (_: Exception) {
        }
        piClient = null
        val msg = if (tracker.calibrator.isCalibrated) {
            "$reason — tap Connect (calibration kept)"
        } else {
            "$reason — tap Connect"
        }
        notifyStatus(buildStatus(connected = false, message = msg))
        updateNotification(msg)
        Log.w(TAG, msg)
    }

    private fun startCameraAnalysis() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            val provider = future.get()
            cameraProvider = provider
            analysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
            analysis?.setAnalyzer(cameraExecutor) { proxy ->
                val mediaImage = proxy.image ?: run {
                    proxy.close()
                    return@setAnalyzer
                }
                val rotation = proxy.imageInfo.rotationDegrees
                val image = InputImage.fromMediaImage(mediaImage, rotation)
                val rotated = rotation == 90 || rotation == 270
                val w = if (rotated) proxy.height else proxy.width
                val h = if (rotated) proxy.width else proxy.height
                frameWidth.set(w)
                tracker.analyze(
                    image,
                    proxy,
                    w,
                    h,
                    rotation,
                    motionAssist,
                ) { result ->
                    lastTrack.set(result)
                    if (piConnected.get() && !calibrating.get()) {
                        notifyStatus(buildStatus(track = result))
                    }
                    proxy.close()
                }
            }
            provider.unbindAll()
            provider.bindToLifecycle(
                this,
                CameraSelector.DEFAULT_BACK_CAMERA,
                analysis!!,
            )
        }, ContextCompat.getMainExecutor(this))
    }

    private fun runCalibration() {
        if (!piConnected.get()) return
        calibrating.set(true)
        val width = frameWidth.get().coerceAtLeast(320)
        tracker.calibrator.begin(width)
        piClient?.sendLine("CALIB_START")

        // 1) 5s prep facing camera → front sample
        if (!runPrepCountdown(getString(R.string.calib_prep_face), FRONT_PREP_SECONDS)) {
            abortCalibration()
            return
        }
        if (!runSamplePhase(BodyCalibrator.Phase.FRONT, getString(R.string.calib_face_hold))) {
            abortCalibration()
            return
        }

        // 2) Beep = turn around, then 3s to get situated → back sample
        CalibrationTone.turnAround()
        notifyStatus(
            buildStatus(
                message = getString(R.string.calib_turn_now),
                calibrating = true,
                calibCountdown = 0,
                calibTitle = getString(R.string.calib_prep_back),
            ),
        )
        if (!runPrepCountdown(getString(R.string.calib_prep_back), BACK_PREP_SECONDS)) {
            abortCalibration()
            return
        }
        if (!runSamplePhase(BodyCalibrator.Phase.BACK, getString(R.string.calib_back_hold))) {
            abortCalibration()
            return
        }

        val ok = tracker.calibrator.finish()
        calibrating.set(false)
        if (ok) {
            CalibStore.save(applicationContext, tracker.calibrator)
            sendCalibOkLine()
            beginStreaming()
            sendTrackingPacket()
            CalibrationTone.calibrationComplete()
            notifyStatus(
                buildStatus(
                    message = getString(R.string.calib_complete),
                    calibrated = true,
                ),
            )
            updateNotification(getString(R.string.notification_tracking))
        } else {
            sendLineOrDrop("CALIB_FAIL")
            restoreCalibFromStore()
            CalibrationTone.calibrationFailed()
            notifyStatus(buildStatus(message = getString(R.string.calib_failed)))
        }
    }

    private fun sendCalibOkLine() {
        val cal = tracker.calibrator
        if (!cal.isCalibrated) return
        val colorFront = cal.targetColorFront
        val colorBack = cal.targetColorBack
        val colorTxt = buildString {
            if (colorFront != null) append(" rgbF=${colorFront.r},${colorFront.g},${colorFront.b}")
            if (colorBack != null) append(" rgbB=${colorBack.r},${colorBack.g},${colorBack.b}")
        }
        sendLineOrDrop(
            "CALIB_OK body=${cal.targetBodyFrac} front=${cal.targetBodyFracFront} " +
                "back=${cal.targetBodyFracBack} center=${cal.targetCenterX}$colorTxt",
        )
    }

    /** Get-in-position countdown — no pose samples collected. */
    private fun runPrepCountdown(title: String, seconds: Int): Boolean {
        for (countdown in seconds downTo 1) {
            if (!piConnected.get()) return false
            piClient?.sendLine("CALIB_PREP $countdown")
            notifyStatus(
                buildStatus(
                    message = getString(R.string.calib_get_ready),
                    calibrating = true,
                    calibCountdown = countdown,
                    calibTitle = title,
                ),
            )
            CalibrationTone.countdownTick()
            if (!sleepSecond()) return false
        }
        return piConnected.get()
    }

    /** Hold-still sampling window — collect pose/color for the current phase. */
    private fun runSamplePhase(phase: BodyCalibrator.Phase, title: String): Boolean {
        tracker.calibrator.beginPhase(phase)
        piClient?.sendLine("CALIB_PHASE ${phase.name.lowercase()}")
        for (countdown in SAMPLE_SECONDS downTo 1) {
            if (!piConnected.get()) return false
            piClient?.sendLine("CALIB $countdown")
            notifyStatus(
                buildStatus(
                    message = getString(R.string.calib_countdown, countdown),
                    calibrating = true,
                    calibCountdown = countdown,
                    calibTitle = title,
                ),
            )
            CalibrationTone.countdownTick()
            val secondEnd = System.currentTimeMillis() + 1000
            while (System.currentTimeMillis() < secondEnd && piConnected.get()) {
                sendCalibPacket()
                Thread.sleep(100)
            }
        }
        tracker.calibrator.endPhase()
        return piConnected.get()
    }

    private fun sleepSecond(): Boolean {
        val end = System.currentTimeMillis() + 1000
        while (System.currentTimeMillis() < end && piConnected.get()) {
            Thread.sleep(50)
        }
        return piConnected.get()
    }

    private fun abortCalibration() {
        calibrating.set(false)
        restoreCalibFromStore()
        sendLineOrDrop("CALIB_FAIL")
        CalibrationTone.calibrationFailed()
        notifyStatus(buildStatus(message = getString(R.string.calib_failed)))
    }

    private fun restoreCalibFromStore() {
        if (!CalibStore.loadInto(applicationContext, tracker.calibrator)) {
            tracker.calibrator.reset()
        }
    }

    private fun beginStreaming() {
        if (!piConnected.get() || streaming.getAndSet(true)) return
        sendExecutor.execute {
            while (streaming.get() && piConnected.get()) {
                if (!calibrating.get()) {
                    sendTrackingPacket()
                }
                Thread.sleep(SEND_INTERVAL_MS)
            }
            streaming.set(false)
        }
    }

    private fun sendCalibPacket() {
        val track = lastTrack.get()
        sendPacketOrDrop(
            CartProtocol.buildPacket(
                steering = 0,
                personDetected = track.detected,
                throttle = 0,
                calibrating = true,
                calibOk = false,
            ),
        )
    }

    private fun sendTrackingPacket() {
        val track = lastTrack.get()
        sendPacketOrDrop(
            CartProtocol.buildPacket(
                steering = track.steering,
                personDetected = track.detected,
                throttle = track.throttle,
                calibrating = false,
                calibOk = tracker.calibrator.isCalibrated,
            ),
        )
    }

    private fun sendPacketOrDrop(packet: ByteArray) {
        val client = piClient ?: return
        if (!client.send(packet)) {
            handleLinkLost()
        }
    }

    private fun sendLineOrDrop(line: String) {
        val client = piClient ?: return
        if (!client.sendLine(line)) {
            handleLinkLost()
        }
    }

    private fun buildStatus(
        connected: Boolean = piConnected.get(),
        message: String = if (connected) {
            if (tracker.calibrator.isCalibrated) "Tracking" else getString(R.string.status_connected)
        } else {
            "Idle"
        },
        gpsAccuracyM: Int? = null,
        track: TrackResult? = null,
        calibrating: Boolean = this.calibrating.get(),
        calibrated: Boolean = tracker.calibrator.isCalibrated,
        calibCountdown: Int = 0,
        calibTitle: String = "",
        visionAssist: Boolean = track?.visionAssist == true,
    ): ServiceStatus {
        return ServiceStatus(
            connected = connected,
            message = message,
            gpsAccuracyM = gpsAccuracyM,
            track = track ?: lastTrack.get(),
            calibrating = calibrating,
            calibrated = calibrated,
            calibCountdown = calibCountdown,
            calibTitle = calibTitle,
            visionAssist = visionAssist,
        )
    }

    private fun notifyStatus(status: ServiceStatus) {
        statusListener?.invoke(status)
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) return
        val channel = NotificationChannel(
            CHANNEL_ID,
            getString(R.string.notification_channel_name),
            NotificationManager.IMPORTANCE_LOW,
        ).apply {
            description = getString(R.string.notification_channel_desc)
            setShowBadge(false)
        }
        val mgr = getSystemService(NotificationManager::class.java)
        mgr.createNotificationChannel(channel)
    }

    private fun buildNotification(text: String): Notification {
        val open = PendingIntent.getActivity(
            this,
            0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE,
        )
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(getString(R.string.notification_title))
            .setContentText(text)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentIntent(open)
            .setOngoing(true)
            .setOnlyAlertOnce(true)
            .setForegroundServiceBehavior(NotificationCompat.FOREGROUND_SERVICE_IMMEDIATE)
            .build()
    }

    private fun updateNotification(text: String) {
        val mgr = getSystemService(NotificationManager::class.java)
        mgr.notify(NOTIFICATION_ID, buildNotification(text))
    }

    override fun onDestroy() {
        streaming.set(false)
        piConnected.set(false)
        gpsTracker.stop()
        motionAssist.stop()
        piClient?.disconnect()
        cameraExecutor.shutdown()
        worker.shutdownNow()
        sendExecutor.shutdownNow()
        tracker.close()
        super.onDestroy()
    }

    companion object {
        private const val TAG = "CartFollowService"
        private const val CHANNEL_ID = "cart_follow"
        private const val NOTIFICATION_ID = 1
        private const val SEND_INTERVAL_MS = 100L
        private const val FRONT_PREP_SECONDS = 5
        private const val BACK_PREP_SECONDS = 3
        private const val SAMPLE_SECONDS = 5

        const val ACTION_START = "com.cartfollow.action.START"
        const val ACTION_CALIBRATE = "com.cartfollow.action.CALIBRATE"
        const val ACTION_STOP = "com.cartfollow.action.STOP"
        const val EXTRA_HOST = "host"
        const val EXTRA_HOLE = "hole"

        fun start(context: Context, host: String, hole: Int) {
            val intent = Intent(context, CartFollowService::class.java).apply {
                action = ACTION_START
                putExtra(EXTRA_HOST, host)
                putExtra(EXTRA_HOLE, hole)
            }
            ContextCompat.startForegroundService(context, intent)
        }

        fun calibrate(context: Context) {
            val intent = Intent(context, CartFollowService::class.java).apply {
                action = ACTION_CALIBRATE
            }
            context.startService(intent)
        }

        fun stop(context: Context) {
            val intent = Intent(context, CartFollowService::class.java).apply {
                action = ACTION_STOP
            }
            context.startService(intent)
        }
    }
}
