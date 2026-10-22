package com.samusynth.questcamera.core

import android.view.Surface

interface ISurfaceProvider {
    fun getSurface() : Surface

    /**
     * Camera2 exposure provenance for the frame submitted to this provider.
     *
     * [sensorTimestampNs] is CaptureResult.SENSOR_TIMESTAMP: the start of
     * exposure in the camera sensor clock. Video providers pair it with the
     * SurfaceTexture timestamp and persist the exact source exposure used for
     * every encoded output frame. Non-video providers do not need to override
     * this hook.
     */
    fun onCaptureResult(
        frameNumber: Long,
        sensorTimestampNs: Long,
        exposureTimeNs: Long,
    ) = Unit
}
