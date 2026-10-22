package com.samusynth.questcamera.core

import android.content.Context
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import com.samusynth.questcamera.helper.CameraPermissionRequestActivity
import java.util.concurrent.Executors

class CameraSessionManager: AutoCloseable {
    private val cameraSurfaceProviders = hashSetOf<ISurfaceProvider>()

    private val handlerThread = HandlerThread("CameraCaptureBackground")
    private var currentCamera: CameraDevice? = null
    private var session: CameraCaptureSession? = null

    private var useCase: Int = CameraDevice.TEMPLATE_STILL_CAPTURE

    override fun close() {
        Log.i(TAG, "Closing camera and cleaning up resources.")

        currentCamera?.close()
        currentCamera = null
        session?.close()
        session = null
        handlerThread.quitSafely()
        if (Thread.currentThread() != handlerThread) {
            try {
                handlerThread.join()
            } catch (e: InterruptedException) {
                Log.e("HandlerThread", "Interrupted while stopping the thread", e)
                Thread.currentThread().interrupt()
            }
        } else {
            // Camera callbacks run on this HandlerThread. Joining the current
            // thread would deadlock the process and prevent Unity from resuming.
            Log.w(TAG, "Camera session closed from its handler thread; skipping self-join.")
        }

        Log.i(TAG, "Resources released.")
    }

    fun isOpen(): Boolean {
        return currentCamera != null
    }

    fun registerSurfaceProvider(surfaceProvider: ISurfaceProvider) {
        cameraSurfaceProviders.add(surfaceProvider)
    }

    fun unregisterSurfaceProvider(surfaceProvider: ISurfaceProvider) {
        cameraSurfaceProviders.remove(surfaceProvider)
    }

    fun setCaptureTemplateFromString(mode: String) {
        useCase = when (mode.uppercase()) {
            "PREVIEW" -> CameraDevice.TEMPLATE_PREVIEW
            "STILL_CAPTURE" -> CameraDevice.TEMPLATE_STILL_CAPTURE
            "RECORD" -> CameraDevice.TEMPLATE_RECORD
            "VIDEO_SNAPSHOT" -> CameraDevice.TEMPLATE_VIDEO_SNAPSHOT
            "ZERO_SHUTTER_LAG" -> CameraDevice.TEMPLATE_ZERO_SHUTTER_LAG
            else -> CameraDevice.TEMPLATE_MANUAL
        }
    }

    fun openCamera(context: Context, cameraManager: CameraManager, cameraId: String) {
        if (!CameraPermissionRequestActivity.checkSelfPermission(context)) {
            Log.w(TAG, "Camera permission not granted. Aborting camera open.")
            return
        }

        if (currentCamera != null) {
            Log.w(TAG, "Camera already open. Skipping openCamera call.")
            return
        }
        
        // Check if handler thread is in TERMINATED state (can't be restarted after being quit)
        // A NEW thread has state NEW (not yet started), a TERMINATED thread has been quit
        // We need to check for TERMINATED specifically, not isAlive (which is false for both NEW and TERMINATED)
        if (handlerThread.state == Thread.State.TERMINATED) {
            Log.e(TAG, "HandlerThread is terminated - this CameraSessionManager cannot be reused. Create a new instance.")
            return
        }

        val cameraCallback = object : CameraDevice.StateCallback() {
            override fun onOpened(camera: CameraDevice) {
                Log.i(TAG, "Camera $cameraId opened successfully.")

                currentCamera = camera

                createCameraSession(camera, cameraId)
            }

            override fun onDisconnected(camera: CameraDevice) {
                if (camera == currentCamera) {
                    Log.w(TAG, "Camera $cameraId disconnected unexpectedly.")

                    close()
                }
            }

            override fun onError(camera: CameraDevice, error: Int) {
                if (camera == currentCamera) {
                    Log.e(TAG, "Camera $cameraId encountered an error (code: $error).")
                    close()
                }
            }
        }

        try {
            Log.d(TAG, "Opening camera $cameraId...")
            cameraManager.openCamera(
                cameraId,
                cameraCallback,
                Handler(handlerThread.apply { start() }.looper)
            )
        } catch (exception: Exception) {
            Log.e(TAG, "Failed to open camera $cameraId: ${exception.message}", exception)
        }
    }

    private fun createCameraSession(camera: CameraDevice, cameraId: String) {
        val targetSurfaceProviders = cameraSurfaceProviders.toList()

        val surfaces = targetSurfaceProviders.map { it.getSurface() }

        if (surfaces.isEmpty()) {
            Log.w(TAG, "No available surfaces found. Closing camera session for camera $cameraId.")
            close()
            return
        }

        val outputConfigs = surfaces.map { OutputConfiguration(it) }

        val sessionCallback = object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(session: CameraCaptureSession) {
                Log.i(TAG, "Capture session configured for camera $cameraId.")
                val requestBuilder = camera.createCaptureRequest(useCase).apply {
                    surfaces.forEach { addTarget(it) }
                    set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, SOURCE_FPS_RANGE)
                }

                val captureCallback = object : CameraCaptureSession.CaptureCallback() {
                    override fun onCaptureCompleted(
                        session: CameraCaptureSession,
                        request: CaptureRequest,
                        result: TotalCaptureResult,
                    ) {
                        val sensorTimestampNs = result.get(CaptureResult.SENSOR_TIMESTAMP)
                        if (sensorTimestampNs == null) {
                            Log.e(TAG, "Camera $cameraId capture result has no SENSOR_TIMESTAMP")
                            return
                        }
                        val exposureTimeNs =
                            result.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: UNKNOWN_EXPOSURE_NS
                        targetSurfaceProviders.forEach { provider ->
                            provider.onCaptureResult(
                                frameNumber = result.frameNumber,
                                sensorTimestampNs = sensorTimestampNs,
                                exposureTimeNs = exposureTimeNs,
                            )
                        }
                    }
                }

                try {
                    session.setRepeatingRequest(
                        requestBuilder.build(),
                        captureCallback,
                        Handler(handlerThread.looper),
                    )
                    this@CameraSessionManager.session = session
                    Log.i(TAG, "Repeating request started for camera $cameraId.")
                } catch (e: Exception) {
                    Log.e(TAG, "Failed to start repeating request: ${e.message}", e)
                    close()
                }
            }

            override fun onConfigureFailed(session: CameraCaptureSession) {
                Log.e(TAG, "Failed to configure capture session for camera $cameraId.")
                close()
            }
        }

        val sessionConfig = SessionConfiguration(
            SessionConfiguration.SESSION_REGULAR,
            outputConfigs,
            Executors.newSingleThreadExecutor(),
            sessionCallback
        )

        try {
            Log.d(TAG, "Creating capture session for camera $cameraId...")
            camera.createCaptureSession(sessionConfig)
        } catch (e: Exception) {
            Log.e(TAG, "Exception during capture session creation: ${e.message}", e)
            close()
        }
    }

    companion object {
        private val TAG = CameraSessionManager::class.java.simpleName
        private const val UNKNOWN_EXPOSURE_NS = -1L

        /**
         * CONTROL_AE_TARGET_FPS_RANGE on every repeating request. The Quest 3S
         * passthrough cameras advertise only [15,15], [30,30] and [60,60]
         * (probed 2026-08-16). At [30,30] the HAL keeps 3 of every 5 slots of a
         * 20 ms lattice, decimated independently per camera, so index-paired
         * left/right frames are 20 ms apart about half the time. At [60,60] it
         * delivers 50 fps — every 20 ms slot — with the two cameras' SENSOR_TIMESTAMPs
         * identical to the nanosecond, at the same ~6 ms exposure. The video
         * provider selects its 30 Hz from that shared lattice on the sensor clock
         * and records this range in its capture report.
         */
        @JvmField
        val SOURCE_FPS_RANGE: Range<Int> = Range(60, 60)
    }
}
