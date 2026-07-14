package com.cartfollow.tracker

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Matches cart_protocol.py / ESP32 usb_data_t framing.
 * Phone sends steering only; Pi LiDAR supplies throttle.
 * Set PERSON_DETECTED_FLAG in button_states when a person is in frame.
 */
object CartProtocol {
    const val START_BYTE: Byte = 0xAA.toByte()
    const val END_BYTE: Byte = 0xBB.toByte()
    const val PERSON_DETECTED_FLAG: Int = 0x80
    const val CALIBRATING_FLAG: Int = 0x40
    const val CALIB_OK_FLAG: Int = 0x20
    const val PI_PORT: Int = 9747

    const val MAX_STEERING = 512
    const val DEADZONE_X = 5

    // Backup distance control (used only when the Pi LiDAR has no valid reading).
    // Torso height as a fraction of frame height at the desired follow distance.
    // Bigger torso = closer. Tune TARGET_BODY_FRAC for your mount + follow distance.
    const val TARGET_BODY_FRAC = 0.33f
    const val BODY_DEADZONE_FRAC = 0.05f
    const val MAX_BACKUP_THROTTLE = 250 // gentler than LiDAR's 400
    const val MIN_THROTTLE = 50

    fun buildPacket(
        steering: Int,
        personDetected: Boolean,
        throttle: Int = 0,
        calibrating: Boolean = false,
        calibOk: Boolean = false,
    ): ByteArray {
        val emergencyStop: Byte = 0
        val cruiseControl: Byte = 0
        val cruiseSpeed: Short = 0
        val followMeMode: Byte = 1
        val turboMode: Byte = 0
        val batteryLevel: Byte = 0
        val buttonStates = (
            (if (personDetected) PERSON_DETECTED_FLAG else 0) or
            (if (calibrating) CALIBRATING_FLAG else 0) or
            (if (calibOk) CALIB_OK_FLAG else 0)
        ).toByte()

        val body = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN)
        body.putShort(throttle.toShort())
        body.putShort(steering.toShort())
        body.put(emergencyStop)
        body.put(cruiseControl)
        body.putShort(cruiseSpeed)
        body.put(followMeMode)
        body.put(turboMode)
        body.put(batteryLevel)
        body.put(buttonStates)

        val payload = body.array()
        var checksum = 0
        for (b in payload) {
            checksum = (checksum + (b.toInt() and 0xFF)) and 0xFF
        }

        return byteArrayOf(START_BYTE) + payload + checksum.toByte() + END_BYTE
    }

    fun steeringFromCenter(personCenterX: Float, frameWidth: Int, targetCenterX: Float? = null): Int {
        if (frameWidth <= 0) return 0
        val center = targetCenterX ?: (frameWidth / 2f)
        val errorX = personCenterX - center
        if (kotlin.math.abs(errorX) <= DEADZONE_X) return 0
        val ratio = errorX / center
        var steering = (ratio * MAX_STEERING).toInt()
        steering = steering.coerceIn(-MAX_STEERING, MAX_STEERING)
        return steering
    }

    fun throttleFromBodyFraction(bodyFrac: Float, targetBodyFrac: Float = TARGET_BODY_FRAC): Int {
        if (bodyFrac <= 0f) return 0
        val error = targetBodyFrac - bodyFrac
        if (kotlin.math.abs(error) <= BODY_DEADZONE_FRAC) return MIN_THROTTLE
        val ratio = error / targetBodyFrac
        var throttle = (ratio * MAX_BACKUP_THROTTLE).toInt()
        if (throttle != 0 && kotlin.math.abs(throttle) < MIN_THROTTLE) {
            throttle = if (throttle > 0) MIN_THROTTLE else -MIN_THROTTLE
        }
        return throttle.coerceIn(-MAX_BACKUP_THROTTLE, MAX_BACKUP_THROTTLE)
    }
}
