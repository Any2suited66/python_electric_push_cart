package com.cartfollow.tracker

import android.graphics.ImageFormat
import androidx.camera.core.ImageProxy
import com.google.mlkit.vision.pose.PoseLandmark
import kotlin.math.max
import kotlin.math.min

/**
 * Samples average shirt color from the torso region (pose landmarks + image crop).
 */
object TorsoColorSampler {
    private const val SAMPLE_GRID = 4
    private const val PADDING_FRAC = 0.08f

    fun torsoBounds(
        leftShoulder: PoseLandmark?,
        rightShoulder: PoseLandmark?,
        leftHip: PoseLandmark?,
        rightHip: PoseLandmark?,
        frameWidth: Int,
        frameHeight: Int,
    ): IntArray? {
        val xs = listOfNotNull(
            leftShoulder?.position?.x,
            rightShoulder?.position?.x,
            leftHip?.position?.x,
            rightHip?.position?.x,
        )
        val ys = listOfNotNull(
            leftShoulder?.position?.y,
            rightShoulder?.position?.y,
            leftHip?.position?.y,
            rightHip?.position?.y,
        )
        if (xs.isEmpty() || ys.isEmpty()) return null

        val padX = frameWidth * PADDING_FRAC
        val padY = frameHeight * PADDING_FRAC
        val left = (xs.min() - padX).toInt().coerceIn(0, frameWidth - 1)
        val right = (xs.max() + padX).toInt().coerceIn(left + 1, frameWidth)
        val top = (ys.min() - padY).toInt().coerceIn(0, frameHeight - 1)
        val bottom = (ys.max() + padY).toInt().coerceIn(top + 1, frameHeight)
        return intArrayOf(left, top, right, bottom)
    }

    fun sampleTorso(
        proxy: ImageProxy,
        bounds: IntArray,
        frameWidth: Int,
        frameHeight: Int,
        rotationDegrees: Int,
    ): RgbColor? {
        if (proxy.format != ImageFormat.YUV_420_888) return null
        val left = bounds[0]
        val top = bounds[1]
        val right = bounds[2]
        val bottom = bounds[3]
        val samples = mutableListOf<RgbColor>()
        val stepX = max(1, (right - left) / SAMPLE_GRID)
        val stepY = max(1, (bottom - top) / SAMPLE_GRID)

        for (fy in top until bottom step stepY) {
            for (fx in left until right step stepX) {
                val (bx, by) = frameToBuffer(fx.toFloat(), fy.toFloat(), rotationDegrees, proxy, frameWidth, frameHeight)
                samplePixel(proxy, bx, by)?.let { samples.add(it) }
            }
        }
        return RgbColor.average(samples)
    }

    private fun frameToBuffer(
        fx: Float,
        fy: Float,
        rotation: Int,
        proxy: ImageProxy,
        frameWidth: Int,
        frameHeight: Int,
    ): Pair<Int, Int> {
        return when (rotation) {
            90 -> Pair(
                fy.toInt().coerceIn(0, proxy.width - 1),
                (frameWidth - fx).toInt().coerceIn(0, proxy.height - 1),
            )
            180 -> Pair(
                (frameWidth - fx).toInt().coerceIn(0, proxy.width - 1),
                (frameHeight - fy).toInt().coerceIn(0, proxy.height - 1),
            )
            270 -> Pair(
                (frameHeight - fy).toInt().coerceIn(0, proxy.width - 1),
                fx.toInt().coerceIn(0, proxy.height - 1),
            )
            else -> Pair(
                fx.toInt().coerceIn(0, proxy.width - 1),
                fy.toInt().coerceIn(0, proxy.height - 1),
            )
        }
    }

    private fun samplePixel(proxy: ImageProxy, x: Int, y: Int): RgbColor? {
        val yPlane = proxy.planes[0]
        val uPlane = proxy.planes[1]
        val vPlane = proxy.planes[2]
        val yRow = yPlane.rowStride
        val uvRow = uPlane.rowStride
        val uvPixel = uPlane.pixelStride
        val yIdx = y * yRow + x
        if (yIdx < 0 || yIdx >= yPlane.buffer.capacity()) return null
        val yVal = yPlane.buffer.get(yIdx).toInt() and 0xFF
        val uvX = x / 2
        val uvY = y / 2
        val uvIdx = uvY * uvRow + uvX * uvPixel
        if (uvIdx < 0 || uvIdx + 1 >= uPlane.buffer.capacity()) return null
        val uVal = uPlane.buffer.get(uvIdx).toInt() and 0xFF
        val vVal = vPlane.buffer.get(uvIdx).toInt() and 0xFF
        return yuvToRgb(yVal, uVal, vVal)
    }

    private fun yuvToRgb(y: Int, u: Int, v: Int): RgbColor {
        val c = y - 16
        val d = u - 128
        val e = v - 128
        val r = ((298 * c + 409 * e + 128) shr 8).coerceIn(0, 255)
        val g = ((298 * c - 100 * d - 208 * e + 128) shr 8).coerceIn(0, 255)
        val b = ((298 * c + 516 * d + 128) shr 8).coerceIn(0, 255)
        return RgbColor(r, g, b)
    }
}
