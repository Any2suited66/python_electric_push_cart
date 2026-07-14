package com.cartfollow.tracker

import android.util.Log
import androidx.camera.core.ImageProxy
import com.google.mlkit.vision.common.InputImage
import com.google.mlkit.vision.pose.Pose
import com.google.mlkit.vision.pose.PoseDetection
import com.google.mlkit.vision.pose.PoseDetector
import com.google.mlkit.vision.pose.PoseLandmark
import com.google.mlkit.vision.pose.defaults.PoseDetectorOptions

data class TrackResult(
    val detected: Boolean,
    val centerX: Float,
    val steering: Int,
    val landmarkCount: Int,
    val throttle: Int = 0,
    val bodyFrac: Float = 0f,
    val calibrating: Boolean = false,
    val calibOk: Boolean = false,
    val colorMatch: Boolean = true,
    val qualityPass: Boolean = true,
    val visionAssist: Boolean = false,
)

class PersonTracker {
    private val detector: PoseDetector = PoseDetection.getClient(
        PoseDetectorOptions.Builder()
            .setDetectorMode(PoseDetectorOptions.STREAM_MODE)
            .build()
    )
    val calibrator = BodyCalibrator()
    private val smoother = TrackSmoother()

    fun analyze(
        image: InputImage,
        imageProxy: ImageProxy?,
        frameWidth: Int,
        frameHeight: Int,
        rotationDegrees: Int,
        motionAssist: PhoneMotionAssist,
        onResult: (TrackResult) -> Unit,
    ) {
        detector.process(image)
            .addOnSuccessListener { pose ->
                onResult(
                    processPose(
                        pose,
                        imageProxy,
                        frameWidth,
                        frameHeight,
                        rotationDegrees,
                        motionAssist,
                    ),
                )
            }
            .addOnFailureListener { e ->
                Log.w(TAG, "Pose detection failed", e)
                onResult(
                    smoother.update(
                        rawDetected = false,
                        centerX = 0f,
                        rawSteering = 0,
                        frameWidth = frameWidth,
                        motionAssist = motionAssist,
                        calibrating = false,
                        calibOk = calibrator.isCalibrated,
                        throttle = 0,
                        bodyFrac = 0f,
                        landmarkCount = 0,
                        colorMatch = false,
                        qualityPass = false,
                    ),
                )
            }
    }

    private fun processPose(
        pose: Pose,
        imageProxy: ImageProxy?,
        frameWidth: Int,
        frameHeight: Int,
        rotationDegrees: Int,
        motionAssist: PhoneMotionAssist,
    ): TrackResult {
        val nose = pose.getPoseLandmark(PoseLandmark.NOSE)
        val leftHip = pose.getPoseLandmark(PoseLandmark.LEFT_HIP)
        val rightHip = pose.getPoseLandmark(PoseLandmark.RIGHT_HIP)
        val leftShoulder = pose.getPoseLandmark(PoseLandmark.LEFT_SHOULDER)
        val rightShoulder = pose.getPoseLandmark(PoseLandmark.RIGHT_SHOULDER)

        val centerX = when {
            leftHip != null && rightHip != null ->
                (leftHip.position.x + rightHip.position.x) / 2f
            nose != null -> nose.position.x
            else -> return smoother.update(
                rawDetected = false,
                centerX = 0f,
                rawSteering = 0,
                frameWidth = frameWidth,
                motionAssist = motionAssist,
                calibrating = calibrator.isCalibrating,
                calibOk = calibrator.isCalibrated,
                throttle = 0,
                bodyFrac = 0f,
                landmarkCount = pose.allPoseLandmarks.size,
                colorMatch = false,
                qualityPass = false,
            )
        }

        val inFrame = centerX > 0 && centerX < frameWidth
        val bodyFrac = torsoFraction(leftShoulder, rightShoulder, leftHip, rightHip, frameHeight)
        val bounds = TorsoColorSampler.torsoBounds(
            leftShoulder, rightShoulder, leftHip, rightHip, frameWidth, frameHeight,
        )
        val torsoColor = if (imageProxy != null && bounds != null) {
            TorsoColorSampler.sampleTorso(
                imageProxy, bounds, frameWidth, frameHeight, rotationDegrees,
            )
        } else {
            null
        }

        if (calibrator.isCalibrating) {
            if (bodyFrac > 0f) {
                calibrator.addSample(bodyFrac, centerX)
            }
            torsoColor?.let { calibrator.addColorSample(it) }
            return TrackResult(
                detected = bodyFrac > 0f && inFrame,
                centerX = centerX,
                steering = 0,
                landmarkCount = pose.allPoseLandmarks.size,
                throttle = 0,
                bodyFrac = bodyFrac,
                calibrating = true,
                colorMatch = true,
                qualityPass = bodyFrac > 0f,
            )
        }

        val landmarksOk = TrackQualityGate.landmarksReliable(
            leftShoulder, rightShoulder, leftHip, rightHip,
        )
        val colorMatch = torsoColor?.let { calibrator.colorMatches(it) } ?: true
        val requireColor = calibrator.isCalibrated &&
            (calibrator.targetColorFront != null || calibrator.targetColorBack != null)
        val qualityPass = inFrame && TrackQualityGate.passes(
            calibrator,
            landmarksOk,
            bodyFrac,
            centerX,
            frameWidth,
            torsoColor,
            requireColor,
        )

        val targetCenter = if (calibrator.isCalibrated) {
            calibrator.targetCenterX
        } else {
            frameWidth / 2f
        }
        val targetBody = if (calibrator.isCalibrated) {
            calibrator.bestTargetBodyFrac(bodyFrac)
        } else {
            CartProtocol.TARGET_BODY_FRAC
        }

        val rawSteering = if (qualityPass) {
            CartProtocol.steeringFromCenter(centerX, frameWidth, targetCenter)
        } else {
            0
        }
        val throttle = if (qualityPass && bodyFrac > 0f) {
            CartProtocol.throttleFromBodyFraction(bodyFrac, targetBody)
        } else {
            0
        }

        return smoother.update(
            rawDetected = inFrame && bodyFrac > 0f && landmarksOk,
            centerX = centerX,
            rawSteering = rawSteering,
            frameWidth = frameWidth,
            motionAssist = motionAssist,
            calibrating = false,
            calibOk = calibrator.isCalibrated,
            throttle = throttle,
            bodyFrac = bodyFrac,
            landmarkCount = pose.allPoseLandmarks.size,
            colorMatch = colorMatch,
            qualityPass = qualityPass,
        )
    }

    private fun torsoFraction(
        leftShoulder: PoseLandmark?,
        rightShoulder: PoseLandmark?,
        leftHip: PoseLandmark?,
        rightHip: PoseLandmark?,
        frameHeight: Int,
    ): Float {
        if (frameHeight <= 0) return 0f
        val shoulderY = averageY(leftShoulder, rightShoulder) ?: return 0f
        val hipY = averageY(leftHip, rightHip) ?: return 0f
        val torsoPx = kotlin.math.abs(hipY - shoulderY)
        return torsoPx / frameHeight
    }

    private fun averageY(a: PoseLandmark?, b: PoseLandmark?): Float? {
        return when {
            a != null && b != null -> (a.position.y + b.position.y) / 2f
            a != null -> a.position.y
            b != null -> b.position.y
            else -> null
        }
    }

    fun resetSmoothing() {
        smoother.reset()
    }

    /** Clears pose smoothing and learned body/color profile. */
    fun resetTracking() {
        smoother.reset()
        calibrator.reset()
    }

    fun close() {
        detector.close()
    }

    companion object {
        private const val TAG = "PersonTracker"
    }
}
