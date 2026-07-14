package com.cartfollow.tracker

import android.content.Context

/** Persists body/color calibration so reconnect does not require re-calib. */
object CalibStore {
    private const val PREFS = "cart_follow_calib"
    private const val KEY_OK = "ok"
    private const val KEY_BODY = "body"
    private const val KEY_FRONT = "front"
    private const val KEY_BACK = "back"
    private const val KEY_CENTER = "center"
    private const val KEY_CF = "cf"
    private const val KEY_CB = "cb"

    fun save(context: Context, cal: BodyCalibrator) {
        if (!cal.isCalibrated) {
            clear(context)
            return
        }
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
            .putBoolean(KEY_OK, true)
            .putFloat(KEY_BODY, cal.targetBodyFrac)
            .putFloat(KEY_FRONT, cal.targetBodyFracFront)
            .putFloat(KEY_BACK, cal.targetBodyFracBack)
            .putFloat(KEY_CENTER, cal.targetCenterX)
            .putString(KEY_CF, encodeColor(cal.targetColorFront))
            .putString(KEY_CB, encodeColor(cal.targetColorBack))
            .apply()
    }

    fun loadInto(context: Context, cal: BodyCalibrator): Boolean {
        val prefs = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        if (!prefs.getBoolean(KEY_OK, false)) return false
        return cal.restore(
            body = prefs.getFloat(KEY_BODY, CartProtocol.TARGET_BODY_FRAC),
            front = prefs.getFloat(KEY_FRONT, CartProtocol.TARGET_BODY_FRAC),
            back = prefs.getFloat(KEY_BACK, CartProtocol.TARGET_BODY_FRAC),
            center = prefs.getFloat(KEY_CENTER, 0f),
            colorFront = decodeColor(prefs.getString(KEY_CF, null)),
            colorBack = decodeColor(prefs.getString(KEY_CB, null)),
        )
    }

    fun clear(context: Context) {
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().clear().apply()
    }

    private fun encodeColor(c: RgbColor?): String? =
        c?.let { "${it.r},${it.g},${it.b}" }

    private fun decodeColor(s: String?): RgbColor? {
        if (s.isNullOrBlank()) return null
        val parts = s.split(",")
        if (parts.size != 3) return null
        return try {
            RgbColor(parts[0].toInt(), parts[1].toInt(), parts[2].toInt())
        } catch (_: Exception) {
            null
        }
    }
}
