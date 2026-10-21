package com.samusynth.questcamera.helper

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import com.samusynth.questcamera.core.EventDispatcher

@SuppressLint("RestrictedApi")
class CameraPermissionRequestActivity : ComponentActivity() {

    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            val cameraGranted = permissions[CAMERA_PERMISSION] == true
            val headsetCameraGranted = permissions[HEADSET_CAMERA_PERMISSION] == true

            permissionResultDispatcher.dispatch(cameraGranted && headsetCameraGranted)
            finish()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        requestPermissionLauncher.launch(
            REQUESTED_PERMISSIONS
        )
    }

    companion object {
        private const val CAMERA_PERMISSION : String = Manifest.permission.CAMERA
        private const val HEADSET_CAMERA_PERMISSION : String = "horizonos.permission.HEADSET_CAMERA"
        /**
         * Horizon OS silently denies UsbManager.requestPermission for a USB video-class device
         * (the rig's wrist cameras) unless the app holds both CAMERA and this, runtime-granted.
         * Asked with the passthrough permissions; passthrough does not depend on it.
         */
        const val USB_CAMERA_PERMISSION : String = "horizonos.permission.USB_CAMERA"
        private val REQUESTED_PERMISSIONS =
            arrayOf(CAMERA_PERMISSION, HEADSET_CAMERA_PERMISSION, USB_CAMERA_PERMISSION)

        val permissionResultDispatcher = EventDispatcher<Boolean>()

        fun checkSelfPermission(context: Context) : Boolean {
            return areAllPermissionsGranted(
                context,
                arrayOf(
                    CAMERA_PERMISSION,
                    HEADSET_CAMERA_PERMISSION
                )
            )
        }

        /** CAMERA and USB_CAMERA, both runtime-granted: what a USB camera's permission needs. */
        @JvmStatic
        fun hasUsbCameraPermissions(context: Context): Boolean =
            areAllPermissionsGranted(context, arrayOf(CAMERA_PERMISSION, USB_CAMERA_PERMISSION))

        fun requestCameraPermissionIfNeeded(context: Context) {
            // An install that already granted passthrough still asks once for USB_CAMERA.
            if (areAllPermissionsGranted(context, REQUESTED_PERMISSIONS)) {
                permissionResultDispatcher.dispatch(true)
                return
            }

            context.startActivity(
                Intent(context, CameraPermissionRequestActivity::class.java)
            )
        }

        fun areAllPermissionsGranted(context: Context, permissions: Array<String>): Boolean {
            return permissions.all {
                ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
            }
        }
    }
}
