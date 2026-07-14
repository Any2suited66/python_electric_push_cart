package com.cartfollow.tracker

import kotlin.math.abs

/**
 * Runs at camera frame rate; send loop reads the latest smoothed [TrackResult].
 */
class TrackSmoother {
    private var smoothCenterX = 0f
    private var smoothSteering = 0
    private var goodStreak = 0
    private var missStreak = 0
    private var outputDetected = false
    private var lastGoodSteering = 0
    private var visionAssistUntilMs = 0L

    fun update(
        rawDetected: Boolean,
        centerX: Float,
        rawSteering: Int,
        frameWidth: Int,
        motionAssist: PhoneMotionAssist,
        calibrating: Boolean,
        calibOk: Boolean,
        throttle: Int,
        bodyFrac: Float,
        landmarkCount: Int,
        colorMatch: Boolean,
        qualityPass: Boolean,
    ): TrackResult {
        if (calibrating) {
            return TrackResult(
                detected = rawDetected,
                centerX = centerX,
                steering = 0,
                landmarkCount = landmarkCount,
                throttle = 0,
                bodyFrac = bodyFrac,
                calibrating = true,
                colorMatch = colorMatch,
                qualityPass = qualityPass,
            )
        }

        val frameGood = rawDetected && qualityPass
        if (frameGood) {
            goodStreak++
            missStreak = 0
            smoothCenterX = if (smoothCenterX == 0f) {
                centerX
            } else {
                smoothCenterX + STEERING_ALPHA * (centerX - smoothCenterX)
            }
            val targetCenter = smoothCenterX
            smoothSteering = CartProtocol.steeringFromCenter(targetCenter, frameWidth, null)
            lastGoodSteering = smoothSteering
            outputDetected = goodStreak >= DET_CONFIRM_FRAMES || outputDetected
            visionAssistUntilMs = System.currentTimeMillis() + VISION_ASSIST_MS
        } else {
            missStreak++
            goodStreak = 0
            if (missStreak >= DET_LOST_FRAMES) {
                outputDetected = false
            }
        }

        var steering = if (outputDetected) smoothSteering else 0
        val assistActive = !outputDetected &&
            System.currentTimeMillis() < visionAssistUntilMs &&
            motionAssist.canAssist()
        if (assistActive) {
            steering = motionAssist.blendSteering(lastGoodSteering)
            outputDetected = true
        }

        return TrackResult(
            detected = outputDetected,
            centerX = if (outputDetected) smoothCenterX else centerX,
            steering = steering,
            landmarkCount = landmarkCount,
            throttle = if (outputDetected) throttle else 0,
            bodyFrac = bodyFrac,
            calibOk = calibOk,
            colorMatch = colorMatch,
            qualityPass = qualityPass,
            visionAssist = assistActive,
        )
    }

    fun reset() {
        smoothCenterX = 0f
        smoothSteering = 0
        goodStreak = 0
        missStreak = 0
        outputDetected = false
        lastGoodSteering = 0
        visionAssistUntilMs = 0L
    }

    companion object {
        private const val STEERING_ALPHA = 0.38f
        private const val DET_CONFIRM_FRAMES = 2
        private const val DET_LOST_FRAMES = 3
        private const val VISION_ASSIST_MS = 800L
    }
}
