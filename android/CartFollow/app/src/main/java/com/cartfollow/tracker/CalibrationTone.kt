package com.cartfollow.tracker

import android.media.AudioManager
import android.media.ToneGenerator

/**
 * Short notification tones for calibration countdown ticks and completion.
 */
object CalibrationTone {
    private var toneGen: ToneGenerator? = null

    fun countdownTick() {
        play(ToneGenerator.TONE_PROP_BEEP, 100, volume = 70)
    }

    /** Front phase finished — turn around for back calibration. */
    fun turnAround() {
        play(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 500, volume = 100)
        try {
            Thread.sleep(200)
        } catch (_: InterruptedException) {
        }
        play(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 500, volume = 100)
    }

    fun calibrationComplete() {
        play(ToneGenerator.TONE_CDMA_CONFIRM, 550, volume = 100)
    }

    fun calibrationFailed() {
        play(ToneGenerator.TONE_CDMA_SOFT_ERROR_LITE, 300, volume = 80)
    }

    private fun play(tone: Int, durationMs: Int, volume: Int) {
        try {
            release()
            toneGen = ToneGenerator(AudioManager.STREAM_NOTIFICATION, volume)
            toneGen?.startTone(tone, durationMs)
        } catch (_: Exception) {
            release()
        }
    }

    fun release() {
        try {
            toneGen?.release()
        } catch (_: Exception) {
        }
        toneGen = null
    }
}
