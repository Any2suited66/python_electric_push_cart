package com.cartfollow.tracker

import kotlin.math.abs
import kotlin.math.sqrt

/** Average RGB sample (0–255 per channel). */
data class RgbColor(val r: Int, val g: Int, val b: Int) {
    fun distanceTo(other: RgbColor): Float {
        val dr = (r - other.r).toFloat()
        val dg = (g - other.g).toFloat()
        val db = (b - other.b).toFloat()
        return sqrt(dr * dr + dg * dg + db * db)
    }

    fun matches(other: RgbColor, tolerance: Int = COLOR_TOLERANCE): Boolean {
        return abs(r - other.r) <= tolerance &&
            abs(g - other.g) <= tolerance &&
            abs(b - other.b) <= tolerance
    }

    fun blend(other: RgbColor): RgbColor {
        return RgbColor(
            (r + other.r) / 2,
            (g + other.g) / 2,
            (b + other.b) / 2,
        )
    }

    companion object {
        const val COLOR_TOLERANCE = 40

        fun average(samples: List<RgbColor>): RgbColor? {
            if (samples.isEmpty()) return null
            return RgbColor(
                samples.map { it.r }.average().toInt(),
                samples.map { it.g }.average().toInt(),
                samples.map { it.b }.average().toInt(),
            )
        }
    }
}
