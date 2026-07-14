package com.cartfollow.tracker

import kotlin.math.abs

/**
 * Learns body shape and shirt color from the front and back during calibration.
 */
class BodyCalibrator {
    enum class Phase { FRONT, BACK }

    var isCalibrating = false
        private set
    var isCalibrated = false
        private set
    var currentPhase: Phase? = null
        private set

    var targetBodyFrac = CartProtocol.TARGET_BODY_FRAC
        private set
    var targetBodyFracFront = CartProtocol.TARGET_BODY_FRAC
        private set
    var targetBodyFracBack = CartProtocol.TARGET_BODY_FRAC
        private set
    var targetCenterX = 0f
        private set
    var targetColorFront: RgbColor? = null
        private set
    var targetColorBack: RgbColor? = null
        private set

    private val frontBodySamples = mutableListOf<Float>()
    private val frontCenterSamples = mutableListOf<Float>()
    private val backBodySamples = mutableListOf<Float>()
    private val backCenterSamples = mutableListOf<Float>()
    private val frontColorSamples = mutableListOf<RgbColor>()
    private val backColorSamples = mutableListOf<RgbColor>()

    fun begin(frameWidth: Int) {
        isCalibrating = false
        isCalibrated = false
        currentPhase = null
        frontBodySamples.clear()
        frontCenterSamples.clear()
        backBodySamples.clear()
        backCenterSamples.clear()
        frontColorSamples.clear()
        backColorSamples.clear()
        targetColorFront = null
        targetColorBack = null
        targetCenterX = frameWidth / 2f
    }

    fun beginPhase(phase: Phase) {
        isCalibrating = true
        currentPhase = phase
    }

    fun addSample(bodyFrac: Float, centerX: Float) {
        if (!isCalibrating || bodyFrac <= 0f) return
        when (currentPhase) {
            Phase.FRONT -> {
                frontBodySamples.add(bodyFrac)
                frontCenterSamples.add(centerX)
            }
            Phase.BACK -> {
                backBodySamples.add(bodyFrac)
                backCenterSamples.add(centerX)
            }
            null -> Unit
        }
    }

    fun addColorSample(color: RgbColor) {
        if (!isCalibrating) return
        when (currentPhase) {
            Phase.FRONT -> frontColorSamples.add(color)
            Phase.BACK -> backColorSamples.add(color)
            null -> Unit
        }
    }

    fun endPhase() {
        isCalibrating = false
        currentPhase = null
    }

    fun finish(): Boolean {
        isCalibrating = false
        currentPhase = null
        if (frontBodySamples.isEmpty() || backBodySamples.isEmpty()) {
            isCalibrated = false
            return false
        }
        targetBodyFracFront = frontBodySamples.average().toFloat()
        targetBodyFracBack = backBodySamples.average().toFloat()
        targetBodyFrac = (targetBodyFracFront + targetBodyFracBack) / 2f
        val centers = frontCenterSamples + backCenterSamples
        targetCenterX = centers.average().toFloat()
        targetColorFront = RgbColor.average(frontColorSamples)
        targetColorBack = RgbColor.average(backColorSamples)
        isCalibrated = true
        return true
    }

    fun bestTargetBodyFrac(currentBodyFrac: Float): Float {
        if (!isCalibrated) return CartProtocol.TARGET_BODY_FRAC
        val frontErr = abs(currentBodyFrac - targetBodyFracFront)
        val backErr = abs(currentBodyFrac - targetBodyFracBack)
        return if (frontErr <= backErr) targetBodyFracFront else targetBodyFracBack
    }

    fun colorMatches(color: RgbColor): Boolean {
        if (!isCalibrated) return true
        val front = targetColorFront
        val back = targetColorBack
        if (front == null && back == null) return true
        val frontOk = front?.matches(color) == true
        val backOk = back?.matches(color) == true
        return frontOk || backOk
    }

    fun reset() {
        begin(0)
        targetBodyFrac = CartProtocol.TARGET_BODY_FRAC
        targetBodyFracFront = CartProtocol.TARGET_BODY_FRAC
        targetBodyFracBack = CartProtocol.TARGET_BODY_FRAC
        targetCenterX = 0f
    }

    /** Restore a previously saved profile (e.g. after reconnect / app restart). */
    fun restore(
        body: Float,
        front: Float,
        back: Float,
        center: Float,
        colorFront: RgbColor?,
        colorBack: RgbColor?,
    ): Boolean {
        if (front <= 0f || back <= 0f) return false
        isCalibrating = false
        currentPhase = null
        frontBodySamples.clear()
        frontCenterSamples.clear()
        backBodySamples.clear()
        backCenterSamples.clear()
        frontColorSamples.clear()
        backColorSamples.clear()
        targetBodyFrac = body
        targetBodyFracFront = front
        targetBodyFracBack = back
        targetCenterX = center
        targetColorFront = colorFront
        targetColorBack = colorBack
        isCalibrated = true
        return true
    }
}
