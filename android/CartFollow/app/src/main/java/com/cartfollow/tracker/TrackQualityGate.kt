package com.cartfollow.tracker

import com.google.mlkit.vision.pose.PoseLandmark
import kotlin.math.abs

/** Pose confidence, calibrated size/lane, and shirt color checks. */
object TrackQualityGate {
    private const val MIN_LIKELIHOOD = 0.55f
    private const val BODY_FRAC_TOLERANCE = 0.14f
    // Intentionally wide: off-center is what steering corrects. A tight center
    // check used to drop the track mid-turn → cart stopped ~1–2s then weaved
    // the other way when the pose re-locked.
    private const val CENTER_TOLERANCE_FRAC = 0.48f

    fun landmarksReliable(
        leftShoulder: PoseLandmark?,
        rightShoulder: PoseLandmark?,
        leftHip: PoseLandmark?,
        rightHip: PoseLandmark?,
    ): Boolean {
        val hips = listOfNotNull(leftHip, rightHip)
        if (hips.isEmpty()) return false
        val shoulders = listOfNotNull(leftShoulder, rightShoulder)
        val hipOk = hips.all { it.inFrameLikelihood >= MIN_LIKELIHOOD }
        val shoulderOk = shoulders.isEmpty() ||
            shoulders.any { it.inFrameLikelihood >= MIN_LIKELIHOOD }
        return hipOk && shoulderOk
    }

    fun profileMatches(
        calibrator: BodyCalibrator,
        bodyFrac: Float,
        centerX: Float,
        frameWidth: Int,
    ): Boolean {
        if (!calibrator.isCalibrated || bodyFrac <= 0f) return true
        val targetBody = calibrator.bestTargetBodyFrac(bodyFrac)
        if (abs(bodyFrac - targetBody) > BODY_FRAC_TOLERANCE) return false
        // Only reject someone far outside the frame lane (wrong person), not
        // normal steering error.
        if (frameWidth > 0) {
            val centerTol = frameWidth * CENTER_TOLERANCE_FRAC
            if (abs(centerX - calibrator.targetCenterX) > centerTol) return false
        }
        return true
    }

    fun passes(
        calibrator: BodyCalibrator,
        landmarksOk: Boolean,
        bodyFrac: Float,
        centerX: Float,
        frameWidth: Int,
        color: RgbColor?,
        requireColor: Boolean,
    ): Boolean {
        if (!landmarksOk || bodyFrac <= 0f) return false
        if (!profileMatches(calibrator, bodyFrac, centerX, frameWidth)) return false
        if (requireColor && color != null && !calibrator.colorMatches(color)) return false
        return true
    }
}
